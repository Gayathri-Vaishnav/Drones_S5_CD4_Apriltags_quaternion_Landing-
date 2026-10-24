"""Generate the project workflow illustration in figures/."""
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

def main():
    labels=["Address +\napproximate beacon","PyBullet camera\nrender","AprilTag ID\n+ solvePnP","α-β-γ pad\ntracker","Velocity-aware\nlanding guidance","Attitude +\nbody thrust","Touchdown\nstate machine"]
    fig,ax=plt.subplots(figsize=(14,3.1)); ax.set_xlim(0,14); ax.set_ylim(0,3); ax.axis("off")
    for i,label in enumerate(labels):
        x=.2+i*1.97
        rect=FancyBboxPatch((x,.95),1.55,1.05,boxstyle="round,pad=.08",facecolor="#e7f3f4",edgecolor="#177e89",linewidth=1.5)
        ax.add_patch(rect); ax.text(x+.775,1.48,label,ha="center",va="center",fontsize=9)
        if i<len(labels)-1: ax.add_patch(FancyArrowPatch((x+1.56,1.48),(x+1.9,1.48),arrowstyle="-|>",mutation_scale=13,color="#555"))
    fig.tight_layout(); out=Path(__file__).resolve().parent/"figures"; out.mkdir(exist_ok=True)
    fig.savefig(out/"pybullet_workflow.png",dpi=170,bbox_inches="tight"); plt.close(fig)
    print(f"Wrote {out/'pybullet_workflow.png'}")

if __name__=="__main__": main()
