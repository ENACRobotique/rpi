#!/usr/bin/env python3

import sys
sys.path.append("../..")
from robot import Robot
import ecal.nanobind_core as ecal_core
from sw.IO.actionneurs import PosTentacle

# === Boucle principale ===
if __name__ == "__main__":
    with Robot() as r:
        # while ecal_core.ok():
        #     pass
        
        r.move(100,0,1000)
        r.actionneurs.HeilG(PosTentacle.BAS)
        #r.actionneurs.GrabG(True)
        r.actionneurs.HeilG(PosTentacle.HAUT)
        r.move(100,0,1000)
        r.actionneurs.HeilG(PosTentacle.BAS)
        #r.actionneurs.GrabG(False)
        r.move(100,0,1000)
        r.move(100,90,1000)
        
        ecal_core.finalize()