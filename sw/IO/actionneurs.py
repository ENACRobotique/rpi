from enum import Enum
from sap_master import SAPMaster
import time
from sts3032 import STS3032

TENTACLE_SPEED = 1741


class PosTentacle(Enum):
    BAS = 0
    HAUT = 1


class Actionneur(Enum):
    AimantBasDroit = 4
    
    ##écran en face de soit, biceps Gauches (tentG) ou droit (tentD), de même pour les pompes
    
    pumpG1 = 40
    pumpG2 = 41
    pumpG3 = 42
    pumpG4 = 43
    
    pumpD1 = 44
    pumpD2 = 45
    pumpD3 = 46
    pumpD4 = 47

    tricepsD = 11
    bicepsD = 0
    tricepsG = 0
    bicepsG = 0


class ValeurActionneur(Enum):
    STSLowSpeed = 3000

    valeurBas = 512
    valeurHaut = 820




class IO_Manager:
    def __init__(self):
        self.sap_master = SAPMaster()
        

    def __repr__(self) -> str:
        return "Robot Enac IOs managment class"

    def initActionneur(self):
        """Passage de tout les actionneurs à leur position de début de match \n"""
        pass


    def read(self, actionneur : Actionneur):
        return self.sap_master.sts3032.readPos(actionneur.value)
        

    def HeilG(self, position : PosTentacle):
        print("Heil")
        if position == PosTentacle.BAS:
            print("BAS")
            #self.sap_master.ax12.move_speed(Actionneur.tricepsG.value, ValeurActionneur.valeurBas.value, TENTACLE_SPEED)
            #self.sap_master.ax12.move_speed(Actionneur.bicepsG.value, ValeurActionneur.valeurBas.value, TENTACLE_SPEED)
        elif position == PosTentacle.HAUT:
            print("HAUT")
            #self.sap_master.ax12.move_speed(Actionneur.tricepsG.value, ValeurActionneur.valeurHaut.value, TENTACLE_SPEED)
            #self.sap_master.ax12.move_speed(Actionneur.bicepsG.value, ValeurActionneur.valeurHaut.value, TENTACLE_SPEED)

        else:
            print("Position inconnue")
            
            
    def HeilD(self, position : PosTentacle):
        print("Heil")
        if position == PosTentacle.BAS:
            print("BAS")
            self.sap_master.ax12.move_speed(Actionneur.tricepsD.value, ValeurActionneur.valeurBas.value, TENTACLE_SPEED)
            #self.sap_master.ax12.move_speed(Actionneur.bicepsD.value, ValeurActionneur.valeurBas.value, TENTACLE_SPEED)
        elif position == PosTentacle.HAUT:
            print("HAUT")
            self.sap_master.ax12.move_speed(Actionneur.tricepsD.value, ValeurActionneur.valeurHaut.value, TENTACLE_SPEED)
            #self.sap_master.ax12.move_speed(Actionneur.bicepsD.value, ValeurActionneur.valeurHaut.value, TENTACLE_SPEED)
        else:
            print("Position inconnue")
        
        
    def GrabG(self,pos):
        if pos == 0:
            self.sap_master.pump.pump(Actionneur.pumpG1.value, 0)
            self.sap_master.pump.valve_use(Actionneur.pumpG1.value)
            self.sap_master.pump.pump(Actionneur.pumpG2.value, 0)
            self.sap_master.pump.valve_use(Actionneur.pumpG2.value)
            self.sap_master.pump.pump(Actionneur.pumpG3.value, 0)
            self.sap_master.pump.valve_use(Actionneur.pumpG3.value)
            self.sap_master.pump.pump(Actionneur.pumpG4.value, 0)
            self.sap_master.pump.valve_use(Actionneur.pumpG4.value)
        elif pos == 1:
            self.sap_master.pump.pump(Actionneur.pumpG1.value, 1)
            self.sap_master.pump.pump(Actionneur.pumpG2.value, 1)
            self.sap_master.pump.pump(Actionneur.pumpG3.value, 1)
            self.sap_master.pump.pump(Actionneur.pumpG4.value, 1)

    def GrabD(self,pos):
        if pos == 0:
            self.sap_master.pump.pump(Actionneur.pumpD1.value, 0)
            self.sap_master.pump.valve_use(Actionneur.pumpD1.value)
            self.sap_master.pump.pump(Actionneur.pumpD2.value, 0)
            self.sap_master.pump.valve_use(Actionneur.pumpD2.value)
            self.sap_master.pump.pump(Actionneur.pumpD3.value, 0)
            self.sap_master.pump.valve_use(Actionneur.pumpD3.value)
            self.sap_master.pump.pump(Actionneur.pumpD4.value, 0)
            self.sap_master.pump.valve_use(Actionneur.pumpD4.value)
        elif pos == 1:
            self.sap_master.pump.pump(Actionneur.pumpD1.value, 1)
            self.sap_master.pump.pump(Actionneur.pumpD2.value, 1)
            self.sap_master.pump.pump(Actionneur.pumpD3.value, 1)
            self.sap_master.pump.pump(Actionneur.pumpD4.value, 1)
            
    def Grab(self,pump,pos):
        if pos == 0:
            self.sap_master.pump.pump(pump.value, 0)
        elif pos == 1:
            self.sap_master.pump.pump(pump.value, 1)

            
            
        
    def Heil(self,id):
        
        pass
  
