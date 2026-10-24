"""Smooth real-time matplotlib telemetry plotter for the drone landing simulation.

Uses matplotlib's FuncAnimation with blit=True for fast, flicker-free updates.
Plots are updated every animation frame (~33 ms) and show a rolling window of
data so the traces stay crisp even during long missions.

Usage (standalone):
    from live_plotter import LivePlotter
    plotter = LivePlotter()
    # inside sim loop:
    plotter.update(time, position, velocity, altitude, horiz_error, phase)
    # after sim:
    plotter.finish()
"""
from __future__ import annotations
import numpy as np
import matplotlib
matplotlib.use("TkAgg")  # interactive backend
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# ── dark colour palette ──────────────────────────────────────────────────────
BG       = "#181a1f"
PANEL    = "#20232a"
GRID     = "#2e3138"
CYAN     = "#00e5ff"
MAGENTA  = "#ff4081"
LIME     = "#76ff03"
AMBER    = "#ffc107"
WHITE    = "#e8eaed"
ORANGE   = "#ff9100"
BLUE     = "#448aff"

# Rolling-window length (number of data points kept visible)
MAX_POINTS = 600   # at ~30 Hz control rate this is ~20 s of data


class LivePlotter:
    """Six-panel real-time telemetry dashboard using matplotlib."""

    def __init__(self, window_title: str = "Drone Live Telemetry"):
        # ── data buffers (deques auto-discard old points) ────────────────
        self.t      = deque(maxlen=MAX_POINTS)
        self.x      = deque(maxlen=MAX_POINTS)
        self.y      = deque(maxlen=MAX_POINTS)
        self.z      = deque(maxlen=MAX_POINTS)
        self.vx     = deque(maxlen=MAX_POINTS)
        self.vy     = deque(maxlen=MAX_POINTS)
        self.vz     = deque(maxlen=MAX_POINTS)
        self.speed  = deque(maxlen=MAX_POINTS)
        self.vx_smooth = deque(maxlen=MAX_POINTS)
        self.vy_smooth = deque(maxlen=MAX_POINTS)
        self.vz_smooth = deque(maxlen=MAX_POINTS)
        self.speed_smooth = deque(maxlen=MAX_POINTS)
        self.herr   = deque(maxlen=MAX_POINTS)
        self.phases = deque(maxlen=MAX_POINTS)

        # ── figure + axes ────────────────────────────────────────────────
        plt.rcParams.update({
            "figure.facecolor": BG, "axes.facecolor": PANEL,
            "axes.edgecolor": GRID, "axes.labelcolor": WHITE,
            "text.color": WHITE, "xtick.color": WHITE, "ytick.color": WHITE,
            "grid.color": GRID, "grid.alpha": 0.5,
            "font.size": 9,
        })

        self.fig, axes = plt.subplots(3, 2, figsize=(14, 8))
        self.fig.canvas.manager.set_window_title(window_title)
        self.fig.subplots_adjust(hspace=0.42, wspace=0.28,
                                 left=0.07, right=0.97, top=0.94, bottom=0.06)

        # Unpack axes
        self.ax_xy, self.ax_alt   = axes[0]
        self.ax_vx, self.ax_vy    = axes[1]
        self.ax_spd, self.ax_err  = axes[2]

        # ── initialise each subplot ──────────────────────────────────────
        # 1) XY track (top-left)
        self.ax_xy.set_title("XY Ground Track", fontsize=10, fontweight="bold")
        self.ax_xy.set_xlabel("X (m)"); self.ax_xy.set_ylabel("Y (m)")
        self.ax_xy.set_aspect("equal", adjustable="datalim")
        self.ax_xy.grid(True, linewidth=0.4)
        self.line_xy,  = self.ax_xy.plot([], [], color=CYAN, lw=1.4, alpha=0.85)
        self.dot_xy,   = self.ax_xy.plot([], [], "o", color=MAGENTA, ms=6, zorder=5)

        # 2) Altitude (top-right)
        self.ax_alt.set_title("Altitude", fontsize=10, fontweight="bold")
        self.ax_alt.set_xlabel("Time (s)"); self.ax_alt.set_ylabel("Z (m)")
        self.ax_alt.grid(True, linewidth=0.4)
        self.line_alt, = self.ax_alt.plot([], [], color=LIME, lw=1.4)

        # 3) Velocity X (mid-left)
        self.ax_vx.set_title("Velocity X", fontsize=10, fontweight="bold")
        self.ax_vx.set_xlabel("Time (s)"); self.ax_vx.set_ylabel("Vx (m/s)")
        self.ax_vx.grid(True, linewidth=0.4)
        self.line_vx, = self.ax_vx.plot([], [], color=CYAN, lw=1.2)

        # 4) Velocity Y (mid-right)
        self.ax_vy.set_title("Velocity Y", fontsize=10, fontweight="bold")
        self.ax_vy.set_xlabel("Time (s)"); self.ax_vy.set_ylabel("Vy (m/s)")
        self.ax_vy.grid(True, linewidth=0.4)
        self.line_vy, = self.ax_vy.plot([], [], color=AMBER, lw=1.2)

        self.ax_vz = self.ax_vy.twinx()
        self.ax_vz.set_ylabel("Vz (m/s)", color=BLUE)
        self.ax_vz.tick_params(axis="y", colors=BLUE)
        self.line_vz, = self.ax_vz.plot([], [], color=BLUE, lw=1.2, alpha=0.9)

        # 5) Speed magnitude (bottom-left)
        self.ax_spd.set_title("Speed |V|", fontsize=10, fontweight="bold")
        self.ax_spd.set_xlabel("Time (s)"); self.ax_spd.set_ylabel("m/s")
        self.ax_spd.grid(True, linewidth=0.4)
        self.line_spd, = self.ax_spd.plot([], [], color=ORANGE, lw=1.4)

        # 6) Horizontal error to pad (bottom-right)
        self.ax_err.set_title("Horizontal Error to Pad", fontsize=10, fontweight="bold")
        self.ax_err.set_xlabel("Time (s)"); self.ax_err.set_ylabel("Error (m)")
        self.ax_err.grid(True, linewidth=0.4)
        self.line_err, = self.ax_err.plot([], [], color=MAGENTA, lw=1.4)

        # Phase text overlay (top-right corner of altitude plot)
        self.phase_text = self.ax_alt.text(
            0.98, 0.92, "", transform=self.ax_alt.transAxes,
            ha="right", va="top", fontsize=11, fontweight="bold",
            color=LIME, bbox=dict(facecolor=BG, edgecolor=GRID,
                                  boxstyle="round,pad=0.3", alpha=0.85))

        # Show the window in non-blocking mode
        self.fig.show()
        plt.pause(0.001)

        self._update_counter = 0
        self._smooth_velocity = None
        self._last_t = None

    # ── public API ───────────────────────────────────────────────────────
    def update(self, t: float, pos, vel, horiz_error: float, phase: str):
        """Push one data point.  pos/vel are 3-element array-likes."""
        pos = np.asarray(pos, dtype=float)
        vel = np.asarray(vel, dtype=float)

        self.t.append(t)
        self.x.append(pos[0]); self.y.append(pos[1]); self.z.append(pos[2])
        raw_speed = float(np.linalg.norm(vel))
        dt = max(0.0, float(t - self._last_t)) if self._last_t is not None else 0.0
        alpha = 1.0 - np.exp(-dt / 0.18) if dt > 0 else 1.0
        self._smooth_velocity = vel.copy() if self._smooth_velocity is None else (
            self._smooth_velocity + alpha * (vel - self._smooth_velocity))
        self._last_t = float(t)
        self.vx.append(vel[0]); self.vy.append(vel[1]); self.vz.append(vel[2])
        self.vx_smooth.append(self._smooth_velocity[0]); self.vy_smooth.append(self._smooth_velocity[1])
        self.vz_smooth.append(self._smooth_velocity[2])
        self.speed.append(raw_speed)
        smooth_speed = float(np.linalg.norm(self._smooth_velocity))
        self.speed_smooth.append(smooth_speed)
        self.herr.append(horiz_error)
        self.phases.append(phase)

        # Redraw every 4th call (~7-8 Hz visual refresh) for smoothness
        self._update_counter += 1
        if self._update_counter % 4 != 0:
            return

        ta = np.array(self.t)

        # XY track
        self.line_xy.set_data(list(self.x), list(self.y))
        if len(self.x) > 0:
            self.dot_xy.set_data([self.x[-1]], [self.y[-1]])
        self.ax_xy.relim(); self.ax_xy.autoscale_view()

        # Altitude
        self.line_alt.set_data(ta, np.array(self.z))
        self.ax_alt.relim(); self.ax_alt.autoscale_view()

        # Velocity X
        self.line_vx.set_data(ta, np.array(self.vx_smooth))
        self.ax_vx.relim(); self.ax_vx.autoscale_view()

        # Velocity Y
        self.line_vy.set_data(ta, np.array(self.vy_smooth))
        self.ax_vy.relim(); self.ax_vy.autoscale_view()
        self.line_vz.set_data(ta, np.array(self.vz_smooth))
        self.ax_vz.relim(); self.ax_vz.autoscale_view()

        # Speed
        self.line_spd.set_data(ta, np.array(self.speed_smooth))
        self.ax_spd.relim(); self.ax_spd.autoscale_view()

        # Horizontal error
        self.line_err.set_data(ta, np.array(self.herr))
        self.ax_err.relim(); self.ax_err.autoscale_view()

        # Phase label
        self.phase_text.set_text(phase.replace("_", " "))

        # Flush to screen without blocking
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def finish(self):
        """Call after the mission ends to keep plots open for inspection."""
        # Final redraw
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        print("[LivePlotter] Mission complete – close the plot window to continue.")

    def close(self):
        """Close the plot window."""
        plt.close(self.fig)
