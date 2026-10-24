"""Run the scene, perception, static/dynamic, controller-ablation and MC studies."""
import argparse, json
from pathlib import Path
import numpy as np
from config import ADDRESSES
from simulation import LandingSimulation, TELEMETRY_FIELDS

OUT=Path(__file__).resolve().parent/"results"

def run_trial(address, seed=1, seconds=70, feedforward=True, plot_name=None, output_dir=None,
              start_position=None, wind_scale=1., wind_mean=None):
    sim=LandingSimulation(gui=False,seed=seed,start_position=start_position); sim.select(address)
    try:
        sim.drone.wind_scale=float(wind_scale)
        if wind_mean is not None:
            sim.drone.wind_mean=np.asarray(wind_mean,dtype=float)
            sim.drone.wind=sim.drone.wind_mean.copy()
        sim.scenario["start_pose_m"]=(list(map(float,start_position)) if start_position is not None else [0.,-7.,.35])
        sim.scenario["wind_scale"] = float(wind_scale)
        sim.scenario["wind_mean_m_s"] = sim.drone.wind_mean.tolist()
        sim.write_manifest(sim.manifest_path)
        sim.drone.enable_feedforward=feedforward
        sim.drone.enable_gain_schedule=feedforward
        sim.guidance.altitude_schedule=feedforward
        for _ in range(int(seconds*120)):
            s=sim.step()
            if s["phase"] in ("LANDED","ABORT"): break
        final=sim.snapshot(); target=sim.world.target_position(address,sim.time)
        err=float(np.linalg.norm(final["position"][:2]-target[:2]))
        fields={name:index for index,name in enumerate(TELEMETRY_FIELDS)}
        count=sum(1 for r in sim.rows if r[fields["tag_id"]]>=0)
        pose_errors=[r[fields["tag_pose_error_m"]] for r in sim.rows
                     if r[fields["tag_id"]]>=0 and r[fields["tag_pose_error_m"]]>=0]
        valid_rows=[r for r in sim.rows if r[fields["tag_id"]]>=0]
        xyz=np.asarray([[r[fields[k]] for k in ("x_m","y_m","z_m")] for r in sim.rows],dtype=float)
        attitudes=np.asarray([[r[fields[k]] for k in ("roll_deg","pitch_deg")] for r in sim.rows],dtype=float)
        tilt_rad=np.arccos(np.clip(np.cos(np.radians(attitudes[:,0]))*np.cos(np.radians(attitudes[:,1])),-1,1)) if attitudes.size else np.asarray([])
        max_tilt=float(np.degrees(np.max(tilt_rad))) if tilt_rad.size else None
        path_length=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum()) if len(xyz)>1 else 0.0
        time_to_acquire=(float(valid_rows[0][fields["time_s"]]) if valid_rows else None)
        final_row=sim.rows[-1] if sim.rows else None
        timing=sim.timing_summary()
        failsafe_events=["timeout"] if final["phase"]=="ABORT" else []
        if output_dir is not None:
            from pathlib import Path
            sim.save_csv(Path(output_dir)/f"address_{address:02d}_seed_{seed:04d}.csv")
        if plot_name and sim.rows:
            import matplotlib.pyplot as plt
            data=np.asarray([r[3:9] for r in sim.rows],dtype=float)
            fig,(ax0,ax1)=plt.subplots(1,2,figsize=(10,4))
            ax0.plot(data[:,0],data[:,1],label="drone"); ax0.plot(data[:,3],data[:,4],"--",label="pad")
            ax0.set(xlabel="East (m)",ylabel="North (m)",title="Horizontal track"); ax0.axis("equal"); ax0.legend()
            ax1.plot([r[0] for r in sim.rows],data[:,2],label="drone z")
            ax1.plot([r[0] for r in sim.rows],data[:,5],"--",label="pad z")
            ax1.set(xlabel="Time (s)",ylabel="Height (m)",title="Altitude profile"); ax1.legend()
            fig.tight_layout(); figdir=Path(__file__).resolve().parent/"figures"; figdir.mkdir(exist_ok=True)
            fig.savefig(figdir/f"pybullet_{plot_name}.png",dpi=150); plt.close(fig)
        return {"address":address,"seed":seed,"phase":final["phase"],"landed":final["phase"]=="LANDED",
                "success":final["phase"]=="LANDED",
                "final_horizontal_error_m":err,"decoded_frames":count,"decoded_fraction":count/max(len(sim.rows),1),
                "mean_tag_pose_error_m":float(np.mean(pose_errors)) if pose_errors else None,
                "frames":len(sim.rows),"simulated_seconds":sim.time,
                "time_to_acquire_s":time_to_acquire,
                "time_to_land_s":float(sim.time) if final["phase"]=="LANDED" else None,
                "touchdown_speed_m_s":float(np.linalg.norm(final["velocity"])) if final["phase"]=="LANDED" else None,
                "max_tilt_deg":max_tilt,"path_length_m":path_length,"energy_j":None,
                "energy_status":"not modeled; battery/power model pending",
                "failsafe_events":failsafe_events,
                "camera_mean_s":timing["camera_render_s"]["mean_s"],
                "controller_mean_s":timing["controller_s"]["mean_s"],
                "physics_step_mean_s":timing["physics_step_s"]["mean_s"],
                "timing":timing,
                "final_contact":bool(final["contact"]),
                "final_sample_time_s":float(final_row[fields["time_s"]]) if final_row else None}
    finally: sim.close()

def evaluate(which="all", missions=40):
    OUT.mkdir(exist_ok=True)
    results={}
    if which in ("all","scene"): results["scene"]={"addresses":len(ADDRESSES),"kinds":sorted({a.kind for a in ADDRESSES.values()}),"physics":"PyBullet rigid-body, 120 Hz"}
    if which in ("all","addressing","perception","static","dynamic","ablation","mc"):
        if which in ("addressing","perception","static","dynamic"):
            nums=list(ADDRESSES) if which=="addressing" else [3 if which=="dynamic" else (5 if which=="perception" else 1)]
            results[which]=[run_trial(n,seed=5,seconds=55,plot_name=which if which in ("static","dynamic") else None) for n in nums]
        if which=="ablation":
            results[which]={"with_feedforward_and_gain_schedule":[run_trial(5,s,55,True) for s in range(6)],
                            "fixed_gain_pid_only":[run_trial(5,s,55,False) for s in range(6)]}
        if which in ("all","mc"):
            if which=="all":
                results["addressing"]=[run_trial(n,seed=10+n,seconds=50) for n in ADDRESSES]
                results["static"]=[run_trial(1,seed=51,seconds=55,plot_name="static_landing")]
                results["dynamic"]=[run_trial(5,seed=52,seconds=55,plot_name="dynamic_tracking")]
                results["perception"]=[run_trial(1,seed=53,seconds=40)]
                results["ablation"]={"with_feedforward_and_gain_schedule":[run_trial(5,s,55,True) for s in range(6)],
                                     "fixed_gain_pid_only":[run_trial(5,s,55,False) for s in range(6)]}
            trials=[run_trial((i%10)+1,seed=i+1,seconds=45) for i in range(missions)]
            results["mc"]={"missions":len(trials),"landed":sum(x["landed"] for x in trials),"trials":trials}
    dest=OUT/f"pybullet_{which}_summary.json"
    dest.write_text(json.dumps(results,indent=2),encoding="utf-8")
    if which in ("all","ablation"):
        import matplotlib.pyplot as plt
        figdir=Path(__file__).resolve().parent/"figures"; figdir.mkdir(exist_ok=True)
        if "ablation" in results:
            ab=results["ablation"]
            if isinstance(ab,dict) and "with_feedforward_and_gain_schedule" in ab:
                names=("Fixed gain PID","Velocity feed-forward + altitude schedule")
                values=(sum(x["landed"] for x in ab["fixed_gain_pid_only"]),sum(x["landed"] for x in ab["with_feedforward_and_gain_schedule"]))
                plt.figure(figsize=(7,4)); plt.bar(names,values,color=("#a3aab4","#22a699")); plt.ylabel("Landed missions (of 6)"); plt.ylim(0,6.5); plt.tight_layout(); plt.savefig(figdir/"pybullet_ablation.png",dpi=150); plt.close()
    if which in ("all","scene"):
        import matplotlib.pyplot as plt
        figdir=Path(__file__).resolve().parent/"figures"; figdir.mkdir(exist_ok=True)
        fig,ax=plt.subplots(figsize=(8,6))
        colors={"ground":"#48a999","vehicle":"#e09f3e","roof":"#8357a5"}
        for a in ADDRESSES.values():
            ax.scatter(a.xyz[0],a.xyz[1],s=90,c=colors[a.kind]); ax.text(a.xyz[0]+.2,a.xyz[1]+.2,f"{a.number} {a.code}",fontsize=8)
        ax.set(title="Ten addressed landing pads",xlabel="East (m)",ylabel="North (m)"); ax.grid(alpha=.25); fig.tight_layout(); fig.savefig(figdir/"pybullet_address_layout.png",dpi=150); plt.close(fig)
    if which=="all":
        from make_diagrams import main as make_workflow
        make_workflow()
    print(json.dumps(results,indent=2)); print(f"Wrote {dest}")

if __name__=="__main__":
    ap=argparse.ArgumentParser(); ap.add_argument("--only",choices=("scene","addressing","perception","static","dynamic","ablation","mc")); ap.add_argument("--missions",type=int,default=40)
    a=ap.parse_args(); evaluate(a.only or "all",a.missions)
