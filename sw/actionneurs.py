from enum import Enum
from gpiozero import Button
import sys
sys.path.append("../")
from drivers.smart_servo.test.ecalServoIO import servoIO
import generated.actionneurs_pb2 as actionneurs_pb
import time
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
    AimantHautDroitGRAB = 1800
    AimantHautDroitDROP = 3000
    AimantHautGaucheGRAB = 1800
    AimantHautGaucheDROP = 700
    
    # MULTITURN FACTOR 2
    AscenseurAimantUP = 3950
    AscenseurAimantDOWN = 110

    
    RentreurIN = 20
    RentreurOUT = 4090

    BrasPinceIN = 450
    BrasPinceOUT = 200
    VerrouPinceLOCK = 2400
    VerrouPinceUNLOCK = 3100

    PlancheDroitUP = 0
    PlancheDroitDOWN = 0
    PlancheGaucheUP = 0
    PlancheGaucheDOWN = 0



fdcGauche = Button(21) # led2
fdcDroite = Button(20) # led1

UP = 1
DOWN = -1 
STOP = 0 

CONSERVE_UP = True
CONSERVE_DOWN = False

INSIDE = True
OUTSIDE = False
class IO_Manager:
    def __init__(self):
        self.Servo_IO = servoIO()
        self.liftG_up = 0
        self.liftD_up = 0
    
    def __repr__(self) -> str:
        return "Robot Enac IOs managment class"

    def initActionneur(self):
        """Passage de tout les actionneurs à leur position de début de match \n"""
        pass
    
    def liftPlancheContinu(self, direction, sync:bool =False):
        """ 
        direction \n
        1 or UP : haut\n
        -1 or DOWN : bas\n
        0 or STOP : pas bouger\n
        """
        self.Servo_IO.setEndless(Actionneur.PlancheDroit.value,True)
        self.Servo_IO.setEndless(Actionneur.PlancheGauche.value,True)

        if direction == 1 :
            if not fdcDroite.is_pressed:
                self.Servo_IO.turn(Actionneur.PlancheDroit.value,1,ValeurActionneur.STSLowSpeed.value)
            if not fdcGauche.is_pressed:
                self.Servo_IO.turn(Actionneur.PlancheGauche.value,0,ValeurActionneur.STSLowSpeed.value)
            
        elif direction == -1:
            self.Servo_IO.turn(Actionneur.PlancheDroit.value,0,ValeurActionneur.STSLowSpeed.value)
            self.Servo_IO.turn(Actionneur.PlancheGauche.value,1,ValeurActionneur.STSLowSpeed.value)

        else :
            self.Servo_IO.turn(Actionneur.PlancheDroit.value,0,0)
            self.Servo_IO.turn(Actionneur.PlancheGauche.value,0,0)
    
    def isLiftUp(self):
        return fdcDroite.is_pressed and fdcGauche.is_pressed
    
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
            
    
    def moveRentreur(self, inside: bool, sync:bool = False):
        self.Servo_IO.setEndless(Actionneur.Rentreur.value,False)
        if inside:
            self.Servo_IO.moveSpeed(Actionneur.Rentreur.value, ValeurActionneur.RentreurIN.value, ValeurActionneur.STSLowSpeed.value)
            
        else:
            self.Servo_IO.moveSpeed(Actionneur.Rentreur.value, ValeurActionneur.RentreurOUT.value, ValeurActionneur.STSLowSpeed.value)
            
        
    def liftConserve(self,up:bool, sync:bool = False):
        self.Servo_IO.setEndless(Actionneur.AscenseurAimant.value,False)
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
    
    def deploy_Macon(self):
        """Deployer l'actionneur Maçon\n
        WARNING : Il faut avoir initialisé les lifts avec les fin de courses avant de l'utiliser !!!"""
        self.deployPince(True)
        self.lockPlanche(False)
        self.grabHighConserve(False)
        self.grabLowConserve(False)
        self.liftPlancheContinu(DOWN) # down
        time.sleep(0.5)
        self.liftPlancheContinu(UP) # down
        # if not self.isLiftUp():
        #    return False
        # self.liftD_up = self.Servo_IO.readPos(Actionneur.PlancheDroit.value)
        # self.liftG_up = self.Servo_IO.readPos(Actionneur.PlancheGauche.value)
        # return True # maçon is deployed

    
    def premieresConserve(self):
        """BLOQUANT"""
        self.grabHighConserve(False)
        self.grabLowConserve(True)
        time.sleep(0.3)
        self.liftConserve(CONSERVE_UP)
        time.sleep(2)
        self.moveRentreur(OUTSIDE)
        time.sleep(0.1)
        self.grabHighConserve(True)
        time.sleep(1.5)
        self.grabLowConserve(False)
        time.sleep(0.25)
        self.liftConserve(CONSERVE_DOWN)
        time.sleep(0.9)

        self.grabLowConserve(True) # temporaire pour eviter de peter le robot
        time.sleep(0.7)

        self.moveRentreur(INSIDE)
        time.sleep(0.3)
        self.grabLowConserve(CONSERVE_DOWN)
    
    def etage2(self):
        self.moveRentreur(OUTSIDE)
        time.sleep(2)
        self.grabHighConserve(False)
        time.sleep(0.3)
        self.moveRentreur(INSIDE)
        time.sleep(0.5)
        self.lockPlanche(False)


        
if __name__ == "__main__":
    jerome = IO_Manager()
    time.sleep(1)
    jerome.deploy_Macon()
    while True:
        pass
    print("Maçon deployed")