from enum import Enum
from ..drivers.smart_servo.test.ecalServoIO import servoIO
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
    STSLowSpeed = 0
    AimantBasDroitGRAB = 0
    AimantBasDroitDROP = 0
    AimantBasGaucheGRAB = 0
    AimantBasGaucheDROP = 0
    AimantHautDroitGRAB = 0
    AimantHautDroitDROP = 0
    AimantHautGaucheGRAB = 0
    AimantHautGaucheDROP = 0
    AscenseurAimantUP = 0
    AscenseurAimantDOWN = 0

    RentreurIN = 0
    RentreurOUT = 0

    BrasPinceIN = 0
    BrasPinceOUT = 0
    VerrouPinceLOCK = 0
    VerrouPinceUNLOCK = 0

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
        
    def liftPlanches(self, up:bool, sync:bool = False):
        """Monter ou descendre les planches\n"""
        if up:
            self.Servo_IO.moveSpeed(Actionneur.PlancheDroit, ValeurActionneur.PlancheDroitUP, ValeurActionneur.STSLowSpeed)
            self.Servo_IO.moveSpeed(Actionneur.PlancheGauche, ValeurActionneur.PlancheGaucheUP, ValeurActionneur.STSLowSpeed)
            
        else:
            self.Servo_IO.moveSpeed(Actionneur.PlancheDroit, ValeurActionneur.PlancheDroitDOWN, ValeurActionneur.STSLowSpeed)
            self.Servo_IO.moveSpeed(Actionneur.PlancheGauche, ValeurActionneur.PlancheGaucheDOWN, ValeurActionneur.STSLowSpeed)
            
        
    def lockPlanche(self, lock:bool, sync:bool = False):
        """ Tenir ou lacher la planche du haut"""
        if lock:
            self.Servo_IO.move(Actionneur.VerrouPince, ValeurActionneur.VerrouPinceLOCK)
            
        else :
            self.Servo_IO.move(Actionneur.VerrouPince, ValeurActionneur.VerrouPinceUNLOCK)
            
        
    def deployPince(self, deploy:bool, sync:bool = False):
        if deploy:
            self.Servo_IO.move(Actionneur.BrasPince, ValeurActionneur.BrasPinceOUT, actionneurs_pb.SmartServo.ServoType.AX12)
            
        else :
            self.Servo_IO.move(Actionneur.BrasPince, ValeurActionneur.BrasPinceIN, actionneurs_pb.SmartServo.ServoType.AX12)
            
    
    def stockConserve(self, stock: bool, sync:bool = False):
        if stock:
            self.Servo_IO.moveSpeed(Actionneur.Rentreur, ValeurActionneur.RentreurIN, ValeurActionneur.STSLowSpeed)
            
        else:
            self.Servo_IO.moveSpeed(Actionneur.Rentreur, ValeurActionneur.RentreurOUT, ValeurActionneur.STSLowSpeed)
            
        
    def liftConserve(self,up:bool, sync:bool = False):
        if up:
            self.Servo_IO.moveSpeed(Actionneur.AscenseurAimant, ValeurActionneur.AscenseurAimantUP, ValeurActionneur.STSLowSpeed)
            
        else:
            self.Servo_IO.moveSpeed(Actionneur.AscenseurAimant, ValeurActionneur.AscenseurAimantDOWN, ValeurActionneur.STSLowSpeed)
            
            
    def grabLowConserve(self, grab : bool, sync:bool = False):
        if grab : 
            self.Servo_IO.move(Actionneur.AimantBasGauche,ValeurActionneur.AimantBasGaucheGRAB)
            self.Servo_IO.move(Actionneur.AimantBasDroit,ValeurActionneur.AimantBasDroitGRAB)
            
        else: 
            self.Servo_IO.move(Actionneur.AimantBasGauche,ValeurActionneur.AimantBasGaucheDROP)
            self.Servo_IO.move(Actionneur.AimantBasDroit,ValeurActionneur.AimantBasDroitDROP)
            
        
    def grabHighConserve(self, grab : bool, sync:bool = False):
        if grab : 
            self.Servo_IO.move(Actionneur.AimantHautGauche,ValeurActionneur.AimantHautGaucheGRAB)
            self.Servo_IO.move(Actionneur.AimantHautDroit,ValeurActionneur.AimantHautDroitGRAB)
            
        else: 
            self.Servo_IO.move(Actionneur.AimantHautGauche,ValeurActionneur.AimantHautGaucheDROP)
            self.Servo_IO.move(Actionneur.AimantHautDroit,ValeurActionneur.AimantHautDroitDROP)
            