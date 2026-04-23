#!/usr/bin/env python3

import sys
import numpy as np
sys.path.append("../..")
from robot import Robot, Caisse
import ecal.nanobind_core as ecal_core
from sw.IO.actionneurs import PosTentacle
from time import sleep,time
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

        #r.resetPosOnLidar()
        t= time()
        r.setTargetPos(Pos(400,1200,np.pi/2),blocking=True,timeout=8)
        print("Aller à 400, 1200 : ",time()-t)
        
        t= time()
        r.align_with_pack(COTE_GAUCHE,timeout=2)
        print("Allignement : ",time()-t)
        #r.resetPosOnLidar()
        t= time()
        r.attraper(COTE_GAUCHE)
        print("Attraper : ",time()-t)
        
        t= time()
        r.setTargetPos(Pos(400,800,np.pi/2),blocking=True,timeout=8)
        print("Aller à 400, 800 : ",time()-t)

        t= time()
        r.relacher(COTE_GAUCHE,Caisse.JAUNE)
        print("Relacher : ",time()-t)

        #r.resetPosOnLidar()
        r.setTargetPos(POS_DEPART,blocking=True,timeout=8)
        print("Retour Nid")

        r.relacher(COTE_GAUCHE,Caisse.TOUT)
        r.actionneurs.initActionneur()
        sleep(2)
        ecal_core.finalize()
        exit()
