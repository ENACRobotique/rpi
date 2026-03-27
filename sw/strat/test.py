#!/usr/bin/env python3

import sys
sys.path.append("../..")
from robot import Robot, PosTentacle
import ecal.nanobind_core as ecal_core
#from sw.IO.actionneurs import PosTentacle
from time import sleep

# === Boucle principale ===
if __name__ == "__main__":
    with Robot() as r:
        r.actionneurs.GrabG(False)
        sleep(0.5)
        # while ecal_core.ok():
        #     pass
        print("pipou")
        r.actionneurs.HeilG(PosTentacle.HAUT)
        sleep(1)
        
        r.move(400,0,1000, blocking=True)
        sleep(0.5)
        print("hey")
        r.actionneurs.HeilG(PosTentacle.BAS)
        print("hey2")
        r.actionneurs.GrabG(True)
        sleep(3)
        
        r.actionneurs.HeilG(PosTentacle.HAUT)
        sleep(1)

        r.move(400,0,1000, blocking=True)
        r.actionneurs.HeilG(PosTentacle.BAS)
        sleep(1)
        r.actionneurs.GrabG(False)
        sleep(3)
        exit(0)

        r.actionneurs.HeilG(PosTentacle.HAUT)
        sleep(1)
        r.move(100,0,1000, blocking=True)
        ecal_core.finalize()