"""Generate the AprilTag marker textures consumed by the PyBullet scene."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from config import ADDRESSES
from world import tag_texture

if __name__=="__main__":
    for n in ADDRESSES:
        tag_texture(n); tag_texture(n+10)
    print(f"Generated {len(ADDRESSES)*2} tag36h11 textures for ten addressed pads.")
