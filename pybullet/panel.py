"""Reference-inspired dark dashboard for the PyBullet landing simulation."""
import tkinter as tk
from datetime import datetime
from pathlib import Path
from PIL import Image, ImageTk
import cv2
import numpy as np
from config import ADDRESSES
from simulation import LandingSimulation

BG = "#17191c"
PANEL = "#202328"
TEXT = "#e4e5e4"
MUTED = "#9b9da0"
GREEN = "#69b65d"

class Dashboard:
    def __init__(self):
        self.sim = LandingSimulation(gui=False)
        self.root = tk.Tk()
        self.root.title("TRANSIT | PyBullet landing simulation")
        self.root.configure(bg=BG)
        self.root.geometry("1600x950")
        self.root.minsize(1280, 760)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        header = tk.Frame(self.root, bg=BG, height=40)
        header.grid(row=0, column=0, sticky="ew", padx=12, pady=(8, 4))
        tk.Label(header, text="TRANSIT", fg="#f1f1ee", bg=BG,
                 font=("Segoe UI", 17, "bold")).pack(side="left")
        tk.Label(header, text="  AUTONOMOUS LANDING  /  PYBULLET CITY", fg=MUTED, bg=BG,
                 font=("Segoe UI", 9, "bold")).pack(side="left", padx=10, pady=(7, 0))
        self.clock_label = tk.Label(header, text="SIM 00:00.0", fg=MUTED, bg=BG,
                                    font=("Consolas", 10))
        self.clock_label.pack(side="right", pady=8)
        tk.Button(header, text="RESET VIEW", command=self.reset_view, bg="#2b3035", fg=TEXT,
                  activebackground="#3a4148", activeforeground=TEXT, bd=0,
                  padx=10, pady=5, font=("Segoe UI", 8, "bold"), cursor="hand2").pack(side="right", padx=12)

        content = tk.Frame(self.root, bg=BG)
        content.grid(row=1, column=0, sticky="nsew", padx=12, pady=4)
        content.rowconfigure(1, weight=1)
        content.columnconfigure(0, weight=1)
        content.columnconfigure(1, minsize=450)
        tk.Label(content, text="CITY VIEW  /  DRAG TO ORBIT  ·  WHEEL TO ZOOM", fg=MUTED, bg=BG,
                 font=("Segoe UI", 9, "bold"), anchor="w").grid(row=0, column=0, sticky="ew", pady=(0, 5))
        self.world_label = tk.Label(content, bg="#35383c", bd=0, highlightthickness=1,
                                    highlightbackground="#464a50")
        self.world_label.grid(row=1, column=0, sticky="nsew", padx=(0, 10))
        self.world_camera = {"yaw":18.0,"pitch":-18.0,"distance":25.0,"target":(2.5,7,1.5)}
        self._world_frame = None
        self._last_world_render = -1.0
        self._drag_origin = None
        self.world_label.bind("<ButtonPress-1>", self.start_orbit)
        self.world_label.bind("<B1-Motion>", self.orbit)
        self.world_label.bind("<MouseWheel>", self.zoom)
        self.world_label.bind("<Button-4>", lambda event: self.zoom_steps(1))
        self.world_label.bind("<Button-5>", lambda event: self.zoom_steps(-1))

        right = tk.Frame(content, bg=PANEL, padx=9, pady=8)
        right.grid(row=0, column=1, rowspan=2, sticky="nsew")
        right.columnconfigure((0, 1), weight=1)
        tk.Label(right, text="ONBOARD CAMERA  /  HD 960×720", fg=TEXT, bg=PANEL,
                 font=("Segoe UI", 10, "bold"), anchor="w").grid(row=0, column=0, sticky="ew")
        self.lock_label = tk.Label(right, text="NO LOCK", fg="#e0a766", bg=PANEL,
                                   font=("Consolas", 9, "bold"))
        self.lock_label.grid(row=0, column=1, sticky="e")
        self.camera_label = tk.Label(right, bg="#08090a", bd=0, highlightthickness=1,
                                     highlightbackground="#45484d")
        self.camera_label.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(6, 12))
        tk.Label(right, text="SELECT LANDING SITE", fg=TEXT, bg=PANEL,
                 font=("Segoe UI", 10, "bold"), anchor="w").grid(row=2, column=0, columnspan=2, sticky="ew", pady=(0, 5))
        self.buttons = {}
        for n, a in ADDRESSES.items():
            row, col = 3 + (n-1)//2, (n-1)%2
            kind = getattr(a, "kind", "ground")
            base = "#17506b" if kind == "ground" else ("#704329" if kind == "vehicle" else "#315c47")
            btn = tk.Button(right, text=f"{n:02d}  {a.name}\n{a.code}   /   {kind.upper()}",
                            command=lambda n=n: self.select(n), bg=base, fg="#f0eee9",
                            activebackground=GREEN, activeforeground="white", bd=0,
                            anchor="w", justify="left", padx=8, pady=5,
                            font=("Segoe UI", 8, "bold"), cursor="hand2")
            btn.grid(row=row, column=col, sticky="ew", padx=3, pady=3)
            self.buttons[n] = (btn, base)
        tk.Label(right, text="Keys 1–9, 0 select sites   •   Simulation runs in real time",
                 fg=MUTED, bg=PANEL, font=("Segoe UI", 8), wraplength=340,
                 justify="left").grid(row=8, column=0, columnspan=2, sticky="w", pady=(8, 0))

        telemetry = tk.Frame(self.root, bg="#0f1113", padx=12, pady=8)
        telemetry.grid(row=2, column=0, sticky="ew", padx=12, pady=(6, 12))
        self.phase_value = self._metric(telemetry, "FLIGHT MODE", "IDLE", 0)
        self.site_value = self._metric(telemetry, "DESTINATION", "Choose a site", 1)
        self.alt_value = self._metric(telemetry, "ALTITUDE", "0.00 m", 2)
        self.error_value = self._metric(telemetry, "PAD OFFSET", "—", 3)
        self.speed_value = self._metric(telemetry, "SPEED", "0.00 m/s", 4)
        self.tags_value = self._metric(telemetry, "TAG LOCK", "NO LOCK", 5)
        for i in range(6): telemetry.columnconfigure(i, weight=1)

        self.root.bind("<Key>", self.key)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh()

    @staticmethod
    def _metric(parent, title, value, column):
        box = tk.Frame(parent, bg="#0f1113")
        box.grid(row=0, column=column, sticky="ew", padx=(0, 12))
        tk.Label(box, text=title, fg="#92969a", bg="#0f1113",
                 font=("Segoe UI", 8, "bold")).pack(anchor="w")
        label = tk.Label(box, text=value, fg="#f0f0ec", bg="#0f1113",
                         font=("Consolas", 11, "bold"))
        label.pack(anchor="w", pady=(2, 0))
        return label

    def select(self, n):
        self.sim.select(n)
        for number, (button, base) in self.buttons.items():
            button.configure(bg=GREEN if number == n else base)

    def reset_view(self):
        self.world_camera.update(yaw=18.0,pitch=-18.0,distance=25.0,target=(2.5,7,1.5))

    def start_orbit(self, event):
        self._drag_origin=(event.x,event.y)

    def orbit(self, event):
        if self._drag_origin is None:
            return
        old_x,old_y=self._drag_origin
        self.world_camera["yaw"]=(self.world_camera["yaw"]+(event.x-old_x)*.45)%360
        self.world_camera["pitch"]=float(np.clip(self.world_camera["pitch"]+(event.y-old_y)*.35,-75,-5))
        self._drag_origin=(event.x,event.y)

    def zoom(self, event):
        direction=1 if event.delta>0 else -1
        self.zoom_steps(direction)

    def zoom_steps(self, direction):
        self.world_camera["distance"]=float(np.clip(self.world_camera["distance"]-direction*1.5,9,55))

    def key(self, event):
        if event.char in "123456789": self.select(int(event.char))
        elif event.char == "0": self.select(10)

    def refresh(self):
        snap = self.sim.step(4)
        camera = self.sim.frame if self.sim.frame is not None else np.zeros((720, 960, 3), np.uint8)
        locked = False
        for d in snap["detections"]:
            match = snap["target"] in (d.tag_id, d.tag_id-10)
            locked = locked or match
            color = (75, 220, 105) if match else (40, 160, 240)
            cv2.polylines(camera, [d.corners.astype("int32")], True, color, 2)
            cv2.putText(camera, f"TAG {d.tag_id}", tuple(np.asarray(d.center_px, dtype="int32")),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2)
        if self._world_frame is None or snap["time"]-self._last_world_render >= .10:
            self._world_frame = self.sim.render_world(**self.world_camera)
            self._last_world_render = snap["time"]
        self._show(self.world_label, self._world_frame, (930, 580))
        self._show(self.camera_label, camera, (420, 315))
        pos, vel = snap["position"], snap["velocity"]
        address = snap["target"]
        a = ADDRESSES.get(address) if address is not None else None
        distance = np.linalg.norm(pos[:2]-self.sim.world.target_position(address, snap["time"])[:2]) if address else None
        self.phase_value.configure(text=snap["phase"].replace("_", " ").upper())
        self.site_value.configure(text=f"{a.code}  {a.name}" if a else "Choose a site")
        self.alt_value.configure(text=f"{pos[2]:.2f} m")
        self.error_value.configure(text=f"{distance:.2f} m" if distance is not None else "—")
        self.speed_value.configure(text=f"{np.linalg.norm(vel):.2f} m/s")
        has_contact = snap["contact"]
        status = "PAD CONTACT" if has_contact else ("LOCKED" if locked else "NO LOCK")
        status_color = GREEN if has_contact or locked else "#e0a766"
        self.tags_value.configure(text=status, fg=status_color)
        self.lock_label.configure(text="PAD CONTACT" if has_contact else ("TAG LOCK" if locked else "NO LOCK"),
                                  fg=status_color)
        self.clock_label.configure(text=f"SIM {snap['time']//60:02.0f}:{snap['time']%60:04.1f}")
        self.root.after(30, self.refresh)

    @staticmethod
    def _show(label, frame, size):
        rgb = cv2.cvtColor(cv2.resize(frame, size, interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
        photo = ImageTk.PhotoImage(Image.fromarray(rgb))
        label.configure(image=photo)
        label.image = photo

    def close(self):
        if self.sim.mission.address is not None and self.sim.rows:
            address = self.sim.mission.address
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = Path("results") / f"dashboard_{address}_{stamp}_telemetry.csv"
            self.sim.save_csv(path)
            print(f"Telemetry saved: {path.resolve()}")
        self.sim.close()
        self.root.destroy()

    def run(self): self.root.mainloop()

if __name__ == "__main__": Dashboard().run()
