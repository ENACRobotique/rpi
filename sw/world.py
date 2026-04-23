import time
import numpy as np

# Dictionnaire des positions de ramassage associé à True si pas encore ramasser par nous
RAMASSAGE_POS =  {"NoixJN":True,"NoixJSW":True,"NoixJES":True,"NoixJSE":True,"NoixJEN":True,
                 "NoixBN":True,"NoixBSE":True,"NoxBWS":True,"NoixBSW":True,"NoixBWN":True}

RAMASSAGE_ANG =  {"NoixJN":np.pi/2,"NoixJSW":np.pi/2,"NoixJES":0,"NoixJSE":0,"NoixJEN":0,
                 "NoixBN":np.pi/2,"NoixBSE":np.pi/2,"NoxBWS":0,"NoixBSW":0,"NoixBWN":0}

# Dictionnaire des positions de depot associé au nombre de caisse posee par nous
DEPOT_POS = {"FrigoJN":0,"FrigoJW":0,"FrigoJS":0,"FrigoJES":0,"FrigoJEN":0,
             "FrigoMidNN":0,"FrigoMidNS":0,"FrigoMidS":0,
             "FrigoBN":0,"FrigoBE":0,"FrigoBS":0,"FrigoBWS":0,"FrigoBWN":0}

DEPOT_ANG = {"FrigoJN":0,"FrigoJW":np.pi/2,"FrigoJS":0,"FrigoJES":0,"FrigoJEN":0,
             "FrigoMidNN":0,"FrigoMidNS":0,"FrigoMidS":0,
             "FrigoBN":0,"FrigoBE":np.pi/2,"FrigoBS":0,"FrigoBWS":0,"FrigoBWN":0,
             "NidJ":np.pi/2,"NidB":np.pi/2}

class World:
    def __init__(self) -> None:
        self.thermo_positioned = False # thermometre place ?
        self.MATCH_DURATION = 100        # match duration
        self.enemy_pos = None           # enemy position if known, else None
        self.matchStartTime: float = -1      # Match start time. negative if match not started
        self.backInZone = False
        self.nid = 0
    
    def time_left(self) -> float:
        if self.matchStartTime < 0:
            return self.MATCH_DURATION
        else:
            return max(0, self.MATCH_DURATION - (time.time() - self.matchStartTime))
    
    def match_started(self) -> bool:
        return self.matchStartTime >= 0
        