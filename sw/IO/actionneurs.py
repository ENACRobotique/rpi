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
    
    ##écran en face de soit, tentacules Gauches (tentG) ou droit (tentD), de même pour les pompes
    tentG1 = 22 # (Tentacle)
    tentG2 = 23
    tentG3 = 20
    tentG4 = 21
    
    tentD1 = 4 
    tentD2 = 8 
    tentD3 = 3 
    tentD4 = 5
    
    pompG1 = 40
    pompG2 = 41
    pompG3 = 42
    pompG4 = 43


class ValeurActionneur(Enum):
    STSLowSpeed = 3000
    
    tentG1Haut = 936
    tentG1Bas = 1921
    tentG2Haut = 3064
    tentG2Bas = 2113
    tentG3Haut = 970 
    tentG3Bas = 1981
    tentG4Haut = 1139
    tentG4Bas = 2044
    
    tentD1Haut = 0
    tentD1Bas = 0
    tentD2Haut = 23
    tentD2Bas = 0
    tentD3Haut = 20
    tentD3Bas = 0
    tentD4Haut = 21
    tentD4Bas = 0




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
        if position == PosTentacle.BAS:
            self.sap_master.sts3032.move_speed(Actionneur.tentG1.value, ValeurActionneur.tentG1Bas.value, TENTACLE_SPEED)
            self.sap_master.sts3032.move_speed(Actionneur.tentG2.value, ValeurActionneur.tentG2Bas.value, TENTACLE_SPEED)
            self.sap_master.sts3032.move_speed(Actionneur.tentG3.value, ValeurActionneur.tentG3Bas.value, TENTACLE_SPEED)
            self.sap_master.sts3032.move_speed(Actionneur.tentG4.value, ValeurActionneur.tentG4Bas.value, TENTACLE_SPEED)
        elif position == PosTentacle.HAUT:
            self.sap_master.sts3032.move_speed(Actionneur.tentG1.value, ValeurActionneur.tentG1Haut.value, TENTACLE_SPEED)
            self.sap_master.sts3032.move_speed(Actionneur.tentG2.value, ValeurActionneur.tentG2Haut.value, TENTACLE_SPEED)
            self.sap_master.sts3032.move_speed(Actionneur.tentG3.value, ValeurActionneur.tentG3Haut.value, TENTACLE_SPEED)
            self.sap_master.sts3032.move_speed(Actionneur.tentG4.value, ValeurActionneur.tentG4Haut.value, TENTACLE_SPEED)
            
        
    def HeilD(self):
        self.sap_master.sts3032.move_speed(Actionneur.tentD1.value, 1000, TENTACLE_SPEED)
        self.sap_master.sts3032.move_speed(Actionneur.tentD2.value, 1000, TENTACLE_SPEED)
        self.sap_master.sts3032.move_speed(Actionneur.tentD3.value, 1000, TENTACLE_SPEED)
        self.sap_master.sts3032.move_speed(Actionneur.tentD4.value, 1000, TENTACLE_SPEED)
        
    def Heil(self,id):
        
        pass
  
