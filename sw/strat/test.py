#!/usr/bin/env python3

import sys
import numpy as np
sys.path.append("../..")
from robot import Robot, Caisse
import ecal.nanobind_core as ecal_core
from sw.IO.actionneurs import PosTentacle
from time import sleep
from common import Pos, Speed, next_path, normalize_angle

POS_DEPART = Pos(400,1800,np.pi/2)

# === Boucle principale ===
if __name__ == "__main__":
    with Robot() as r:
        r.actionneurs.GrabG(False)
        r.actionneurs.moveG(PosTentacle.HAUT)
        r.resetPos(POS_DEPART)
        
        sleep(1)

        r.setTargetPos(Pos(400,1200,np.pi/2))
        print("Zone de ramassage")
        sleep(5)
        r.align_with_pack(False)
        sleep(2)
        r.actionneurs.moveG(PosTentacle.BAS)
        sleep(0.5)
        r.actionneurs.GrabG(True)
        sleep(1)
        r.actionneurs.moveG(PosTentacle.HAUT)
        sleep(1)
        r.setTargetPos(Pos(400,800,np.pi/2))
        sleep(5)
        print("Zone de Dépose")
        r.actionneurs.moveG(PosTentacle.BAS)
        sleep(2)
        r.release(False,Caisse.JAUNE)
        sleep(2)
        r.actionneurs.moveG(PosTentacle.HAUT)
        sleep(2)
        
        #r.setTargetPos(Pos(400,400,-np.pi))
        #sleep(3)
        #r.align_with_pack(True)



        r.setTargetPos(POS_DEPART)
        sleep(5)
        print("Retour Nid")
        r.actionneurs.moveG(PosTentacle.BAS)
        sleep(2)
        r.actionneurs.GrabG(False)
        sleep(2)
        ecal_core.finalize()
        exit()
