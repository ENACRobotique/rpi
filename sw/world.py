import time
import numpy as np
from robot import Robot
from common import Pos
import threading

SEUIL_CONSIDERATION = 200 # en mm


class RamassagePoint :
    def __init__(self,nav_point:str,angle:float,pos_milieu:Pos):
        self.nav_point = nav_point
        self.angle = angle
        self.pos_milieu = pos_milieu
        self.ramasse = 1. # Indique s'il y a des caisse => 10 = on est sur qur qu'il y a des caisses, => 0 on est sur qu'il y en a pas
        self.temps_ennemie_passe = 0


class DepotPoint :
    def __init__(self,nav_point:str,angle:float,pos_milieu:Pos):
        self.nav_point = nav_point
        self.angle = angle
        self.pos_milieu = pos_milieu
        self.contient = []
        self.temps_ennemie_passe = 0

POINT_RAMASSAGE = {"NoixJN": RamassagePoint("NoixJN",-np.pi/2,Pos(175,1200,0)),
                     "NoixJSW": RamassagePoint("NoixJSW",-np.pi/2,Pos(175,400,0)),
                     "NoixJES": RamassagePoint("NoixJES",np.pi,Pos(1150,800,0)),
                     "NoixJSE": RamassagePoint("NoixJSE",0,Pos(1100,175,0)),
                     "NoixJEN": RamassagePoint("NoixJEN",0,Pos(1150,800,0)),
                     "NoixBN": RamassagePoint("NoixBN",-np.pi/2,Pos(3000-175,1200,0)),
                     "NoixBSE": RamassagePoint("NoixBSE",-np.pi/2,Pos(3000-175,400,0)),
                     "NoixBWS": RamassagePoint("NoixBWS",np.pi,Pos(3000-1150,800,0)),
                     "NoixBSW": RamassagePoint("NoixBSW",0,Pos(3000-1100,175,0)),
                     "NoixBWN": RamassagePoint("NoixBWN",0,Pos(3000-1150,800,0))}

POINT_DEPOT = {"FrigoJN": DepotPoint("FrigoJN",0,Pos(1250,1450,0)),
                           "FrigoJW": DepotPoint("FrigoJW",-np.pi/2,Pos(100,800,0)),
                           "FrigoJS": DepotPoint("FrigoJS",0,Pos(700,100,0)),
                           "FrigoJES": DepotPoint("FrigoJES",np.pi,Pos(800,800,0)),
                           "FrigoJEN": DepotPoint("FrigoJEN",0,Pos(800,800,0)),
                           "FrigoMidNN": DepotPoint("FrigoMidNN",0,Pos(1500,1000,0)),
                           "FrigoMidNS": DepotPoint("FrigoMidNS",np.pi,Pos(1500,1000,0)),
                           "FrigoMidS": DepotPoint("FrigoMidS",0,Pos(1500,100,0)),
                           "FrigoBN":DepotPoint("FrigoBN",0,Pos(3000-1250,1450,0)),
                           "FrigoBE":DepotPoint("FrigoBE",-np.pi/2,Pos(3000-100,800,0)),
                           "FrigoBS":DepotPoint("FrigoBS",0,Pos(3000-700,100,0)),
                           "FrigoBWS":DepotPoint("FrigoBWS",np.pi,Pos(3000-800,800,0)),
                           "FrigoBWN":DepotPoint("FrigoBWN",0,Pos(3000-800,800,0)),
                           "NidJ":DepotPoint("NidJ",-np.pi/2,Pos(300,1750,0)),
                           "NidB":DepotPoint("NidB",-np.pi/2,Pos(3000-300,1750,0))}

class World:
    def __init__(self, robot: Robot) -> None:
        self.robot = robot
        self.thermo_positioned = False # thermometre place ?
        self.MATCH_DURATION = 100        # match duration
        self.enemy_pos = None           # enemy position if known, else None
        self.matchStartTime: float = -1      # Match start time. negative if match not started
        self.backInZone = False
        self.nid = 0
        self.main_match_action_done = False
        self.thread_track_ennemie = threading.Thread(target=self.track_ennemie,daemon=True)
        self.thread_track_ennemie.start()

    def time_left(self) -> float:
        if self.matchStartTime < 0:
            return self.MATCH_DURATION
        else:
            return max(0, self.MATCH_DURATION - (time.time() - self.matchStartTime))
    
    def match_started(self) -> bool:
        return self.matchStartTime >= 0

    def track_ennemie(self):
        print("[WORLD] Thread OPENING")
        total_tick = 0
        while True:
            time.sleep(0.1)
            ep = self.robot.ennemiePos
            if ep is not None:
                # On regarde s'il est proche d'un point de ramassage
                for nav_point in POINT_RAMASSAGE.keys():
                    if ep.distance(POINT_RAMASSAGE[nav_point].pos_milieu) < SEUIL_CONSIDERATION:
                        if nav_point == "NoixJES":
                            POINT_RAMASSAGE["NoixJEN"].temps_ennemie_passe +=1
                        if nav_point == "NoixJEN":
                            POINT_RAMASSAGE["NoixJES"].temps_ennemie_passe +=1
                        
                        if nav_point == "NoixBES":
                            POINT_RAMASSAGE["NoixBEN"].temps_ennemie_passe +=1
                        if nav_point == "NoixBEN":
                            POINT_RAMASSAGE["NoixBES"].temps_ennemie_passe +=1

                        
                        POINT_RAMASSAGE[nav_point].temps_ennemie_passe +=1 # On augmente le temps passe dans le point interet
                        if POINT_RAMASSAGE[nav_point].temps_ennemie_passe > 50  and POINT_RAMASSAGE[nav_point].ramasse > 0.5:
                            POINT_RAMASSAGE[nav_point].ramasse = 0.5
                            #print(f"***********  [{nav_point}] : Il y a peut etre plus de caisse ici :(  *****************")

                for nav_point in POINT_DEPOT.keys():
                    if ep.distance(POINT_DEPOT[nav_point].pos_milieu) < SEUIL_CONSIDERATION:
                        
                        if nav_point == "FrigoJES":
                            POINT_DEPOT["FrigoJEN"].temps_ennemie_passe +=1
                        if nav_point == "FrigoJEN":
                            POINT_DEPOT["FrigoJES"].temps_ennemie_passe +=1
                        
                        if nav_point == "FrigoBWS":
                            POINT_DEPOT["FrigoBWN"].temps_ennemie_passe +=1
                        if nav_point == "FrigoBWN":
                            POINT_DEPOT["FrigoBWS"].temps_ennemie_passe +=1

                        if nav_point == "FrigoMidNS":
                            POINT_DEPOT["FrigoMidNN"].temps_ennemie_passe +=1
                        if nav_point == "FrigoMidNN":
                            POINT_DEPOT["FrigoMidNS"].temps_ennemie_passe +=1

                        POINT_DEPOT[nav_point].temps_ennemie_passe +=1 # On augmente le temps passe dans le point interet

                