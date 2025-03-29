from enum import Enum
import sys
sys.path.append("../")
from drivers.smart_servo.test.ecalServoIO import servoIO
import generated.actionneurs_pb2 as actionneurs_pb
class Actionneur(Enum):
    #todo : Droit = la droite du robot par rapport à son axe X
    AimantBasDroit = 4
    AimantBasGauche = 3
    AscenseurAimant = 5
    
    AimantHautDroit = 6 
    AimantHautGauche = 7
    Rentreur = 8

    PlancheDroit = 1
    PlancheGauche = 2
    VerrouPince = 9
    BrasPince = 11 # AX

    AscenseurBanderolle = 10

class ValeurActionneur(Enum):
    STSLowSpeed = 3000
    AimantBasDroitGRAB = 3100
    AimantBasDroitDROP = 3800
    AimantBasGaucheGRAB = 2950
    AimantBasGaucheDROP = 2285
    AimantHautDroitGRAB = 1900
    AimantHautDroitDROP = 3000
    AimantHautGaucheGRAB = 1700
    AimantHautGaucheDROP = 700
    AscenseurAimantUP = 0
    AscenseurAimantDOWN = 0

    RentreurIN = 0
    RentreurOUT = 0

    BrasPinceIN = 450
    BrasPinceOUT = 200
    VerrouPinceLOCK = 2300
    VerrouPinceUNLOCK = 3500

    PlancheDroitUP = 0
    PlancheDroitDOWN = 0
    PlancheGaucheUP = 0
    PlancheGaucheDOWN = 0

class IO_Manager:
    def __init__(self):
        self.Servo_IO = servoIO()
    
    def __repr__(self) -> str:
        return "Robot Enac IOs managment class"

    def initActionneur(self):
        """Passage de tout les actionneurs à leur position de début de match \n"""
        pass
    
    def liftPlancheContinu(self, direction, sync:bool =False):
        self.Servo_IO.setEndless(Actionneur.PlancheDroit.value,True)
        self.Servo_IO.setEndless(Actionneur.PlancheGauche.value,True)
        if direction == 1 :
            self.Servo_IO.turn(Actionneur.PlancheDroit.value,1,ValeurActionneur.STSLowSpeed.value)
            self.Servo_IO.turn(Actionneur.PlancheGauche.value,0,ValeurActionneur.STSLowSpeed.value)
            
        elif direction == -1:
            self.Servo_IO.turn(Actionneur.PlancheDroit.value,0,ValeurActionneur.STSLowSpeed.value)
            self.Servo_IO.turn(Actionneur.PlancheGauche.value,1,ValeurActionneur.STSLowSpeed.value)

        else :
            self.Servo_IO.turn(Actionneur.PlancheDroit.value,0,0)
            self.Servo_IO.turn(Actionneur.PlancheGauche.value,0,0)
    
    def stockConserveContinu(self, direction, sync:bool =False):
        self.Servo_IO.setEndless(Actionneur.Rentreur.value,True)
        if direction == 1 :
            self.Servo_IO.turn(Actionneur.Rentreur.value,0,ValeurActionneur.STSLowSpeed.value)

        elif direction == -1:
            self.Servo_IO.turn(Actionneur.Rentreur.value,1,ValeurActionneur.STSLowSpeed.value)

        else :
            self.Servo_IO.turn(Actionneur.Rentreur.value,0,0)

    def liftConserveContinu(self, direction, sync:bool =False):
        self.Servo_IO.setEndless(Actionneur.AscenseurAimant.value,True)
        if direction == 1 :
            self.Servo_IO.turn(Actionneur.AscenseurAimant.value,0,ValeurActionneur.STSLowSpeed.value)

        elif direction == -1:
            self.Servo_IO.turn(Actionneur.AscenseurAimant.value,1,ValeurActionneur.STSLowSpeed.value)

        else :
            self.Servo_IO.turn(Actionneur.AscenseurAimant.value,0,0)


    def liftPlanches(self, up:bool, sync:bool = False):
        """Monter ou descendre les planches\n"""
        if up:
            self.Servo_IO.moveSpeed(Actionneur.PlancheDroit.value, ValeurActionneur.PlancheDroitUP.value, ValeurActionneur.STSLowSpeed.value)
            self.Servo_IO.moveSpeed(Actionneur.PlancheGauche.value, ValeurActionneur.PlancheGaucheUP.value, ValeurActionneur.STSLowSpeed.value)
            
        else:
            self.Servo_IO.moveSpeed(Actionneur.PlancheDroit.value, ValeurActionneur.PlancheDroitDOWN.value, ValeurActionneur.STSLowSpeed.value)
            self.Servo_IO.moveSpeed(Actionneur.PlancheGauche.value, ValeurActionneur.PlancheGaucheDOWN.value, ValeurActionneur.STSLowSpeed.value)
            
        
    def lockPlanche(self, lock:bool, sync:bool = False):
        """ Tenir ou lacher la planche du haut"""
        if lock:
            self.Servo_IO.move(Actionneur.VerrouPince.value, ValeurActionneur.VerrouPinceLOCK.value)
            
        else :
            self.Servo_IO.move(Actionneur.VerrouPince.value, ValeurActionneur.VerrouPinceUNLOCK.value)
            
        
    def deployPince(self, deploy:bool, sync:bool = False):
        if deploy:
            self.Servo_IO.moveSpeed(Actionneur.BrasPince.value, ValeurActionneur.BrasPinceOUT.value,100, actionneurs_pb.SmartServo.ServoType.AX12)
            
        else :
            self.Servo_IO.moveSpeed(Actionneur.BrasPince.value, ValeurActionneur.BrasPinceIN.value,100, actionneurs_pb.SmartServo.ServoType.AX12)
            
    
    def stockConserve(self, stock: bool, sync:bool = False):
        if stock:
            self.Servo_IO.moveSpeed(Actionneur.Rentreur.value, ValeurActionneur.RentreurIN.value, ValeurActionneur.STSLowSpeed.value)
            
        else:
            self.Servo_IO.moveSpeed(Actionneur.Rentreur.value, ValeurActionneur.RentreurOUT.value, ValeurActionneur.STSLowSpeed.value)
            
        
    def liftConserve(self,up:bool, sync:bool = False):
        if up:
            self.Servo_IO.moveSpeed(Actionneur.AscenseurAimant.value, ValeurActionneur.AscenseurAimantUP.value, ValeurActionneur.STSLowSpeed.value)
            
        else:
            self.Servo_IO.moveSpeed(Actionneur.AscenseurAimant.value, ValeurActionneur.AscenseurAimantDOWN.value, ValeurActionneur.STSLowSpeed.value)
            
            
    def grabLowConserve(self, grab : bool, sync:bool = False):
        if grab : 
            self.Servo_IO.move(Actionneur.AimantBasGauche.value,ValeurActionneur.AimantBasGaucheGRAB.value)
            self.Servo_IO.move(Actionneur.AimantBasDroit.value,ValeurActionneur.AimantBasDroitGRAB.value)
            
        else: 
            self.Servo_IO.move(Actionneur.AimantBasGauche.value,ValeurActionneur.AimantBasGaucheDROP.value)
            self.Servo_IO.move(Actionneur.AimantBasDroit.value,ValeurActionneur.AimantBasDroitDROP.value)
            
        
    def grabHighConserve(self, grab : bool, sync:bool = False):
        if grab : 
            self.Servo_IO.move(Actionneur.AimantHautGauche.value,ValeurActionneur.AimantHautGaucheGRAB.value)
            self.Servo_IO.move(Actionneur.AimantHautDroit.value,ValeurActionneur.AimantHautDroitGRAB.value)
            
        else: 
            self.Servo_IO.move(Actionneur.AimantHautGauche.value,ValeurActionneur.AimantHautGaucheDROP.value)
            self.Servo_IO.move(Actionneur.AimantHautDroit.value,ValeurActionneur.AimantHautDroitDROP.value)
            
