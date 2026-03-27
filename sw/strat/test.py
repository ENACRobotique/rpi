#!/usr/bin/env python3

import sys
import numpy as np
sys.path.append("../..")
from robot import Robot, Caisse
import ecal.nanobind_core as ecal_core
from sw.IO.actionneurs import PosTentacle
from time import sleep
from common import Pos, Speed, next_path, normalize_angle

POS_DEPART = Pos(400,1800,-np.pi/2)

# === Boucle principale ===
if __name__ == "__main__":
    with Robot() as r:
        r.actionneurs.GrabD(False)
        r.actionneurs.moveD(PosTentacle.HAUT)
        r.resetPos(POS_DEPART)
        
        sleep(1)

        r.setTargetPos(Pos(400,1200,-np.pi/2))
        sleep(2)
        r.align_with_pack(True)
        sleep(2)
        r.actionneurs.moveD(PosTentacle.BAS)
        print("a")
        r.actionneurs.GrabD(True)
        sleep(1)
        r.actionneurs.moveD(PosTentacle.HAUT)
        r.setTargetPos(Pos(300,800,-np.pi/2))
        sleep(2)
        r.actionneurs.moveD(PosTentacle.BAS)
        r.release(True,Caisse.JAUNE)
        ecal_core.finalize()