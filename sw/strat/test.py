#!/usr/bin/env python3

import sys
import numpy as np
sys.path.append("../..")
from robot import Robot, Caisse
import ecal.nanobind_core as ecal_core
from sw.IO.actionneurs import PosTentacle
from time import sleep,time
from common import Pos, Speed, next_path, normalize_angle

POS_DEPART = Pos(500,1000,-np.pi/2)

# === Boucle principale ===
if __name__ == "__main__":
    with Robot() as r:
        r.actionneurs.initActionneur()
        r.resetPos(POS_DEPART)
        t = time()
        
        sleep(1)

        r.setTargetPos(Pos(200,200,0),blocking=True,timeout=8)
        r.move(150,np.pi,blocking=True,timeout=2)
        r.resetPos(Pos(100,200,0))
        r.brasThermo()
        r.move(500,0,blocking=True,timeout=8)

        r.actionneurs.initActionneur()

        ecal_core.finalize()
        print("Execute en ",time()-t)
        exit()
