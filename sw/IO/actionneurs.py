from enum import Enum
from sap_master import SAPMaster
import time
from sts3032 import STS3032

TENTACLE_SPEED = 300


class PosTentacle(Enum):
    BAS = 0
    HAUT = 1
    RETOURNE = 2
    #POUSSE = 2
    DROP = 3
    THERMO = 4


class Actionneur(Enum):
    # Écran à l'arrière, triceps à l'arrière, biceps à l'avant

    pumpD1 = 40
    pumpD2 = 41
    pumpD3 = 42
    pumpD4 = 43
    
    pumpG1 = 44
    pumpG2 = 46
    pumpG3 = 45
    pumpG4 = 47

    tricepsD = 20
    bicepsD = 5
    tricepsG = 7
    bicepsG = 11

POMPES_DROITES = [Actionneur.pumpD1, Actionneur.pumpD2, Actionneur.pumpD3, Actionneur.pumpD4]
POMPES_GAUCHES = [Actionneur.pumpG1, Actionneur.pumpG2, Actionneur.pumpG3, Actionneur.pumpG4]


VALEURS_ACTIONNEURS = {PosTentacle.BAS.value: 512, PosTentacle.DROP.value: 550, PosTentacle.RETOURNE.value: 950, PosTentacle.HAUT.value: 820, PosTentacle.THERMO.value: 666}


class IO_Manager:
    def __init__(self):
        self.sap_master = SAPMaster()
        
    def __repr__(self) -> str:
        return "Robot Enac IOs managment class"

    def initActionneur(self):
        """Passage de tout les actionneurs à leur position de début de match \n"""
        self.moveG(PosTentacle.HAUT)
        self.moveD(PosTentacle.HAUT)
        self.GrabG(False)
        self.GrabD(False)

    def ready(self,act,val,seuil):
        pos = self.sap_master.ax12.read_pos(act)
        if type(pos) == int:
            return (abs(pos - val) < seuil)
        else :
            return False


    def moveG(self, position : PosTentacle,timeout=1):
        debut = time.time()
        self.sap_master.ax12.move_speed(Actionneur.tricepsG.value, VALEURS_ACTIONNEURS[position.value], TENTACLE_SPEED)
        self.sap_master.ax12.move_speed(Actionneur.bicepsG.value,  VALEURS_ACTIONNEURS[position.value], TENTACLE_SPEED)
        while((time.time()-debut < timeout) 
              and not(self.ready(Actionneur.bicepsG.value,VALEURS_ACTIONNEURS[position.value],50) 
                  and self.ready(Actionneur.tricepsG.value,VALEURS_ACTIONNEURS[position.value],50))):
            time.sleep(0.1)

        
    def moveD(self, position : PosTentacle,timeout=1):
        debut = time.time()
        self.sap_master.ax12.move_speed(Actionneur.tricepsD.value, VALEURS_ACTIONNEURS[position.value], TENTACLE_SPEED)
        self.sap_master.ax12.move_speed(Actionneur.bicepsD.value,  VALEURS_ACTIONNEURS[position.value], TENTACLE_SPEED)
        while((time.time()-debut < timeout) 
              and not(self.ready(Actionneur.bicepsD.value,VALEURS_ACTIONNEURS[position.value],50) 
                  and self.ready(Actionneur.tricepsD.value,VALEURS_ACTIONNEURS[position.value],50))):
            time.sleep(0.1)


    def GrabG(self, grab: bool):
        if grab:
            self.sap_master.pump.pump(Actionneur.pumpG1.value, 1)
            self.sap_master.pump.pump(Actionneur.pumpG2.value, 1)
            self.sap_master.pump.pump(Actionneur.pumpG3.value, 1)
            self.sap_master.pump.pump(Actionneur.pumpG4.value, 1)
        else:
            self.sap_master.pump.pump(Actionneur.pumpG1.value, 0)
            self.sap_master.pump.valve_use(Actionneur.pumpG1.value)
            self.sap_master.pump.pump(Actionneur.pumpG2.value, 0)
            self.sap_master.pump.valve_use(Actionneur.pumpG2.value)
            self.sap_master.pump.pump(Actionneur.pumpG3.value, 0)
            self.sap_master.pump.valve_use(Actionneur.pumpG3.value)
            self.sap_master.pump.pump(Actionneur.pumpG4.value, 0)
            self.sap_master.pump.valve_use(Actionneur.pumpG4.value)

    def GrabD(self, grab: bool):
        if grab:
            self.sap_master.pump.pump(Actionneur.pumpD1.value, 1)
            self.sap_master.pump.pump(Actionneur.pumpD2.value, 1)
            self.sap_master.pump.pump(Actionneur.pumpD3.value, 1)
            self.sap_master.pump.pump(Actionneur.pumpD4.value, 1)
        else:
            self.sap_master.pump.pump(Actionneur.pumpD1.value, 0)
            self.sap_master.pump.valve_use(Actionneur.pumpD1.value)
            self.sap_master.pump.pump(Actionneur.pumpD2.value, 0)
            self.sap_master.pump.valve_use(Actionneur.pumpD2.value)
            self.sap_master.pump.pump(Actionneur.pumpD3.value, 0)
            self.sap_master.pump.valve_use(Actionneur.pumpD3.value)
            self.sap_master.pump.pump(Actionneur.pumpD4.value, 0)
            self.sap_master.pump.valve_use(Actionneur.pumpD4.value)

    def Grab(self, pump: Actionneur, grab: bool):
        self.sap_master.pump.pump(pump.value, grab)
        if not grab :
            self.sap_master.pump.valve_use(pump.value)
