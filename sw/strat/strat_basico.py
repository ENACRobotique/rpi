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

COTE_DROIT = True
COTE_GAUCHE = False

# === Boucle principale ===
if __name__ == "__main__":
    with Robot() as r:
        r.actionneurs.initActionneur()
        r.resetPos(POS_DEPART)
        sleep(1)

        r.setTargetPos(Pos(400,1300,np.pi/2),blocking=True,timeout=8)

        r.setTargetPos(Pos(1150,1025,np.pi),blocking=True,timeout=8)
        print("Zone de ramassage")
        r.align_with_pack(COTE_GAUCHE,timeout=2)
        r.attraper(COTE_GAUCHE)

        r.setTargetPos(Pos(1100,450,0),blocking=True,timeout=8)
        print("Zone de ramassage")
        r.align_with_pack(COTE_DROIT,timeout=2)
        r.attraper(COTE_DROIT)

        r.setTargetPos(Pos(700,400,0),blocking=True,timeout=8)
        print("Zone de Dépose")
        r.relacher(True,Caisse.JAUNE)

        r.setTargetPos(Pos(500,800,-np.pi/2),blocking=True,timeout=8)
        print("Zone de Dépose")
        r.relacher(False,Caisse.JAUNE)

        r.setTargetPos(POS_DEPART,blocking=True,timeout=8)
        print("Retour Nid")
        r.relacher(COTE_DROIT,Caisse.TOUT)
        r.relacher(COTE_GAUCHE,Caisse.TOUT)
        r.actionneurs.initActionneur()
        sleep(5)

        ecal_core.finalize()
        exit()