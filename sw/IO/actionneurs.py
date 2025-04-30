from enum import Enum
import sys
sys.path.append("../../")
from drivers.smart_servo.ecalServoIO import servoIO
from generated import actionneurs_pb2 as actionneurs_pb
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

    PlancheDELTA = 2600

    
servoPosError = 20 # on prend large

UP = True
DOWN = False

INSIDE = True
OUTSIDE = False

class IO_Manager:
    def __init__(self):
        self.Servo_IO = servoIO()
        
        #Handling Planche servos
        self.liftCalibrated = False
        self.liftG_init = -1
        self.liftD_init = -1
        self.liftG_up = -1
        self.liftD_up = -1
        self.liftG_down = -1
        self.liftD_down = -1

    def __repr__(self) -> str:
        return "Robot Enac IOs managment class"

    def initActionneur(self):
        """Passage de tout les actionneurs à leur position de début de match \n"""
        pass

    def calibrateLift(self):
        """ Calibrating will make the servo back to first turn !\n"""
        self.Servo_IO.setEndless(Actionneur.PlancheGauche.value, False)
        self.Servo_IO.setEndless(Actionneur.PlancheDroit.value, False)
        g = 0
        d = 0
        fg = False # Calibration fail flags
        fd = False # Calibration fail flags
        self.liftG_init = -1
        self.liftD_init = -1

        while self.liftG_init == -1:
            g = g + 1
            self.liftG_init = self.Servo_IO.readPos(Actionneur.PlancheGauche.value)
            if g > 5: # In case of timeout reading, number of checks are arbitrary
                print("Cannot calibrate liftG, check connections")
                fg = True
                break

        while self.liftD_init == -1:
            d = d + 1
            self.liftD_init = self.Servo_IO.readPos(Actionneur.PlancheDroit.value)
            if d > 5: # In case of timeout reading, number of checks are arbitrary
                print("Cannot calibrate liftD, check connections")
                fd = True
                break

        self.liftD_up = self.liftD_init//4+100
        self.liftD_down = self.liftD_init//4+ValeurActionneur.PlancheDELTA.value+200
        
        self.liftG_up = self.liftG_init//4 +ValeurActionneur.PlancheDELTA.value-100
        self.liftG_down = self.liftG_init//4
        
        self.liftCalibrated = True
        if not fg or not fd:
            print("Lift calibration sucessfull !")

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
        """Monter ou descendre les planches\n
        Si fdc non calibrés la fonction ne fera rien"""
        if self.liftCalibrated:
            if up:
                self.Servo_IO.moveSpeed(Actionneur.PlancheGauche.value, self.liftG_up, 4000)
                self.Servo_IO.moveSpeed(Actionneur.PlancheDroit.value, self.liftD_up, 4000)     
            else:
                self.Servo_IO.moveSpeed(Actionneur.PlancheGauche.value, self.liftG_down, 4000)
                self.Servo_IO.moveSpeed(Actionneur.PlancheDroit.value, self.liftD_down, 4000)                  
        
    def lockPlanche(self, lock:bool, sync:bool = False):
        """ Tenir ou lâcher la planche du haut"""
        if lock:
            self.Servo_IO.move(Actionneur.VerrouPince.value, ValeurActionneur.VerrouPinceLOCK.value)
            
        else :
            self.Servo_IO.move(Actionneur.VerrouPince.value, ValeurActionneur.VerrouPinceUNLOCK.value)
            
        
    def deployPince(self, deploy:bool, sync:bool = False):
        """Déployer ou rentrer la pince"""
        if deploy:
            self.Servo_IO.move(Actionneur.BrasPince.value, ValeurActionneur.BrasPinceOUT.value, actionneurs_pb.SmartServo.ServoType.AX12)
            
        else :
            self.Servo_IO.move(Actionneur.BrasPince.value, ValeurActionneur.BrasPinceIN.value, actionneurs_pb.SmartServo.ServoType.AX12)
            
    
    def moveRentreur(self, inside: bool, sync:bool = False):
        self.Servo_IO.setEndless(Actionneur.Rentreur.value, False)
        if inside:
            self.Servo_IO.move(Actionneur.Rentreur.value, ValeurActionneur.RentreurIN.value)
            
        else:
            self.Servo_IO.move(Actionneur.Rentreur.value, ValeurActionneur.RentreurOUT.value)
            
        
    def liftConserve(self,up:bool, sync:bool = False):
        self.Servo_IO.setEndless(Actionneur.AscenseurAimant.value, False)
        if up:
            self.Servo_IO.move(Actionneur.AscenseurAimant.value, ValeurActionneur.AscenseurAimantUP.value)
            
        else:
            self.Servo_IO.move(Actionneur.AscenseurAimant.value, ValeurActionneur.AscenseurAimantDOWN.value)
            
            
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
    
    def deployMacon(self):
        """Deployer l'actionneur Maçon\n
         Il faut Calibrer l'ascenceur planche avant !!!\n
         NON BLOQUANT
         """
        self.deployPince(True)
        self.lockPlanche(False)
        self.grabHighConserve(False)
        self.grabLowConserve(False)
        self.liftPlanches(DOWN)

    def ramasseGradin(self):
        """BLOQUANT"""
        self.liftPlanches(UP) #Soulève les planches
        time.sleep(0.5)
        self.grabHighConserve(False)        # lache les préhenseur du haut
        self.grabLowConserve(True)          # sort les préhenseur du bas
        time.sleep(0.3)
        self.liftConserve(UP)      # monte les conserves
        time.sleep(2)
        self.moveRentreur(OUTSIDE)          # sort le rentreur
        time.sleep(0.1)
        self.grabHighConserve(True)         # attrape par le haut
        time.sleep(1.5)
        self.grabLowConserve(False)         # lache par le bas
        time.sleep(0.25)
        self.lockPlanche(True)              # attrape la planche du haut
        self.liftConserve(DOWN)    # descend l'ascenseur à conserve
        time.sleep(0.5)

        self.grabLowConserve(True)          # temporaire pour eviter de peter le robot
        time.sleep(0.7)

        self.moveRentreur(INSIDE)           # rentre les conserves
        time.sleep(0.3)
        self.grabLowConserve(False) # lache les préhenseur bas
    
    def construitGradin(self):
        """BLOQUANT"""
        self.liftPlanches(DOWN)          # Descend la planche du 1er etage
        time.sleep(2.5)
        self.moveRentreur(OUTSIDE)          # Sort les conserves
        time.sleep(1.8)
        self.grabHighConserve(False)        # Lache les conserves
        time.sleep(0.3)
        self.moveRentreur(INSIDE)           # Rentre le rentreur
        time.sleep(0.5)
        self.lockPlanche(False)             # lache la planche du haut
        time.sleep(0.5)
        self.grabLowConserve(False)         # lache les conserves du bas

