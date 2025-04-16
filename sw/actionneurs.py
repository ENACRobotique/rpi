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

    PlancheDroitDELTA = 0
    PlancheGaucheDELTA = 0

    
servoPosError = 20 # on prend large

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
        
        #Handling Planche servos
        self.fdcGauche = Button(21) # led2
        self.fdcDroite = Button(20) # led1
        self.fdcGauche.when_pressed = self.stopLiftG # tester si ça marche en mono ou continu 
        self.fdcDroite.when_pressed = self.stopLiftD # cad pas un "while_pressed"
        self.liftGCalibrated = False
        self.liftDCalibrated = False
        
        self.liftG_up = None
        self.liftD_up = None

    def __repr__(self) -> str:
        return "Robot Enac IOs managment class"

    def initActionneur(self):
        """Passage de tout les actionneurs à leur position de début de match \n"""
        pass

    def stopLiftG(self):
        """Stops planche lifterG rotation\n
        Calibration will also take effect on first function call only if the
        corresponding button is pressed\n"""
        self.Servo_IO.setEndless(Actionneur.PlancheGauche.value,True)
        self.Servo_IO.turn(Actionneur.PlancheGauche.value,0,0)
        if not self.liftGCalibrated:
            if self.fdcGauche.is_pressed: # we make sure twice that the button is actually pressed
                self.liftGCalibrated = True
                self.Servo_IO.setEndless(Actionneur.PlancheGauche.value, False)
                self.liftG_up = self.Servo_IO.readPos(Actionneur.PlancheGauche.value)
                self.Servo_IO.move(Actionneur.PlancheGauche.value, self.liftG_up-ValeurActionneur.PlancheGaucheDELTA.value)
                
                
                
    
    def stopLiftD(self):
        """Stops planche lifterD rotation\n
        Calibration will also take effect on first function call only if the
        corresponding button is pressed\n"""
        self.Servo_IO.setEndless(Actionneur.PlancheDroit.value,True)
        self.Servo_IO.turn(Actionneur.PlancheDroit.value,0,0)
        if not self.liftDCalibrated:
            if self.fdcDroite.is_pressed: # we make sure twice that the button is actually pressed
                self.liftDCalibrated = True
                self.Servo_IO.setEndless(Actionneur.PlancheDroit.value, False)
                self.liftD_up = self.Servo_IO.readPos(Actionneur.PlancheDroit.value)
                self.Servo_IO.move(Actionneur.PlancheDroit.value, self.liftD_up-ValeurActionneur.PlancheDroitDELTA.value)
                
                
                
    
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
            if not self.fdcDroite.is_pressed:
                self.Servo_IO.turn(Actionneur.PlancheDroit.value,1,ValeurActionneur.STSLowSpeed.value)
            if not self.fdcGauche.is_pressed:
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


    def liftUpPlanches(self, up:bool, sync:bool = False):
        """Monter ou descendre les planches\n
        Si fdc non calibrés la fonction ne fera rien"""
        self.Servo_IO.setEndless(Actionneur.PlancheDroit.value, False)
        self.Servo_IO.setEndless(Actionneur.PlancheGauche.value, False)
        if (self.liftD_up is not None) and (self.liftG_up is not None) :
            if up:
                self.Servo_IO.move(Actionneur.PlancheDroit.value, self.liftD_up)
                self.Servo_IO.move(Actionneur.PlancheGauche.value, self.liftG_up)
                
            else:
                self.Servo_IO.move(Actionneur.PlancheDroit.value, self.liftD_up-ValeurActionneur.PlancheDroitDELTA.value)
                self.Servo_IO.move(Actionneur.PlancheGauche.value, self.liftG_up-ValeurActionneur.PlancheGaucheDELTA.value)
                
        
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
         Déclenche l'initialisation des valeurs de l'actionneur planche !!!
         BLOQUANT
         """
        self.liftDCalibrated = False
        self.liftGCalibrated = False
        self.deployPince(True)
        self.lockPlanche(False)
        self.grabHighConserve(False)
        self.grabLowConserve(False)
        self.liftPlancheContinu(DOWN)
        time.sleep(0.5)
        self.liftPlancheContinu(UP)# When the button (fdc) will be pressed this will trigger calibration 
                                   # The lift will then go down and stop moving

    def ramasseGradin(self):
        """BLOQUANT"""
        self.liftUpPlanches(True) #Soulève les planches
        time.sleep(0.2)
        self.grabHighConserve(False)        # lache les préhenseur du haut
        self.grabLowConserve(True)          # sort les préhenseur du bas
        time.sleep(0.3)
        self.liftConserve(CONSERVE_UP)      # monte les conserves
        time.sleep(2)
        self.moveRentreur(OUTSIDE)          # sort le rentreur
        time.sleep(0.1)
        self.grabHighConserve(True)         # attrape par le haut
        time.sleep(1.5)
        self.grabLowConserve(False)         # lache par le bas
        time.sleep(0.25)
        self.lockPlanche(True)              # attrape la planche du haut
        self.liftConserve(CONSERVE_DOWN)    # descend l'ascenseur à conserve
        time.sleep(0.5)

        self.grabLowConserve(True)          # temporaire pour eviter de peter le robot
        time.sleep(0.7)

        self.moveRentreur(INSIDE)           # rentre les conserves
        time.sleep(0.3)
        self.grabLowConserve(False) # lache les préhenseur bas
    
    def construitGradin(self):
        """BLOQUANT"""
        self.liftUpPlanches(False)          # Descen la planche du 1er etage
        time.sleep(1.5)
        self.moveRentreur(OUTSIDE)          # Sort les conserves
        time.sleep(2)
        self.grabHighConserve(False)        # Lache les conserves
        time.sleep(0.3)
        self.moveRentreur(INSIDE)           # Rentre le rentreur
        time.sleep(0.5)
        self.lockPlanche(False)             # lache la planche du haut
        time.sleep(0.5)
        self.grabLowConserve(False)         # lache les conserves du bas


#### Only for abstraction ####
    def isLiftUp(self):
        """Return true if both servo are up"""
        a = abs(self.Servo_IO.readPos(Actionneur.PlancheDroit.value) - self.liftD_up)  < servoPosError
        b = abs(self.Servo_IO.readPos(Actionneur.PlancheGauche.value) - self.liftG_up) < servoPosError
        return a and b
    
    def isLiftDown(self):
        """Return true if both servo are down"""
        a = abs(self.Servo_IO.readPos(Actionneur.PlancheDroit.value) - (self.liftD_up-ValeurActionneur.PlancheDroitDELTA.value))  < servoPosError
        b = abs(self.Servo_IO.readPos(Actionneur.PlancheGauche.value) - (self.liftG_up-ValeurActionneur.PlancheDroitDELTA.value)) < servoPosError
        return a and b
    
    def isConserveUp(self):
        a = abs(self.Servo_IO.readPos(Actionneur.AscenseurAimant.value) - (ValeurActionneur.AscenseurAimantUP.value))  < servoPosError
        return a
    
    def isConserveDown(self):
        a = abs(self.Servo_IO.readPos(Actionneur.AscenseurAimant.value) - (ValeurActionneur.AscenseurAimantDOWN.value))  < servoPosError
        return a
    
    def isRentreurIN(self):
        a = abs(self.Servo_IO.readPos(Actionneur.Rentreur.value) - (ValeurActionneur.RentreurIN.value))  < servoPosError
        return a
    
    def isRentreurOUT(self):
        a = abs(self.Servo_IO.readPos(Actionneur.Rentreur.value) - (ValeurActionneur.RentreurOUT.value))  < servoPosError
        return a
    

# if __name__ == "__main__":
#     jerome = IO_Manager()
#     time.sleep(1)
#     jerome.deploy_Macon()