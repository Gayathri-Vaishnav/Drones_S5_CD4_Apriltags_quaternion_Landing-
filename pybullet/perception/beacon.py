"""Approximate address beacon fix used only to bring the camera into search range."""
from dataclasses import dataclass
import numpy as np
from config import ADDRESSES

@dataclass(frozen=True)
class BeaconFix:
    address: int
    position: np.ndarray
    sigma_m: float

def approximate_fix(address, rng, sigma_m=.18):
    if address not in ADDRESSES: raise ValueError("Unknown pad address")
    center=np.asarray(ADDRESSES[address].xyz,dtype=float)
    return BeaconFix(address,center+rng.normal(0,sigma_m,3),sigma_m)
