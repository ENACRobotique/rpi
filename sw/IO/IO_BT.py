import py_trees
import sys
sys.path.append("../..")
from sw.IO.actionneurs import *
from robot import *
from bt_essentials import get_bb_robot

##### Behavior tree pour l'automatisation #####

class LiftPlanche(py_trees.behaviour.Behaviour):
    def __init__(self, up:bool):
        pos = "DOWN"
        if up:
            pos = "UP"
        super().__init__(name=f"Lift {pos} planche")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.up = up
        self.done = False
        self.pos = pos

    def initialise(self) -> None:
        print(f"Lift {self.pos} planche")
        self.robot.actionneurs.liftPlanches(self.up)
        
    def update(self):
        if self.robot.actionneurs.Servo_IO.isMoving(Actionneur.PlancheGauche.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.PlancheDroit.value):
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS



class LiftConserve(py_trees.behaviour.Behaviour):
    def __init__(self, pos:ValeurActionneur):
        super().__init__(name=f"Lift {pos.name} Conserve")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.pos = pos

    def initialise(self) -> None:
        print(f"Lift {self.pos} Conserve")
        self.robot.actionneurs.liftConserve(self.pos)

    def update(self):
        if self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AscenseurAimant.value):
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS
    
class MoveRentreur(py_trees.behaviour.Behaviour):
    def __init__(self, position:bool):
        pos = "OUTSIDE"
        if position:
            pos = "INSIDE"
        super().__init__(name=f"Rentreur going {pos}")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.position = position
        self.done = False
        self.pos = pos

    def initialise(self) -> None:
        print(f"Rentreur going {self.pos}")
        self.robot.actionneurs.moveRentreur(self.position)

    def update(self):
        if self.robot.actionneurs.Servo_IO.isMoving(Actionneur.Rentreur.value):
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

class GrabHighConserve(py_trees.behaviour.Behaviour):
    def __init__(self, grab : bool):
        stat = "Dropping"
        if grab:
            stat = "Grabbing"
        super().__init__(name=f"{stat} High Conserve")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.grab = grab
        self.done = False
        self.stat= stat

    def initialise(self) -> None:
        print(f"{self.stat} High Conserve")
        self.robot.actionneurs.grabHighConserve(self.grab)

    def update(self):
        if self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AimantHautGauche.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AimantHautDroit.value):
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS



class GrabLowConserve(py_trees.behaviour.Behaviour):
    def __init__(self, grab : bool):
        stat = "Dropping"
        if grab:
            stat = "Grabbing"
        super().__init__(name=f"{stat} Low Conserve")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.grab = grab
        self.done = False
        self.stat = stat
    
    def initialise(self) -> None:
        print(f"{self.stat} Low Conserve")
        self.robot.actionneurs.grabLowConserve(self.grab) 

    def update(self):
        if self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AimantBasGauche.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AimantBasDroit.value):
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS
    
class LockPlanche(py_trees.behaviour.Behaviour):
    def __init__(self, lock : bool):
        stat = "Unlocking"
        if lock:
            stat = "Locking"
        super().__init__(name=f"{stat} Upper Planche")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.lock = lock
        self.done = False
        self.stat = stat

    def initialise(self) -> None:
        print(f"{self.stat} Upper Planche")
        self.robot.actionneurs.lockPlanche(self.lock)

    def update(self):
        if self.robot.actionneurs.Servo_IO.isMoving(Actionneur.VerrouPince.value):
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS
    
class DeployMacon(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name=f"Deploying Macon")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.deployed = False

    def initialise(self) -> None:
        print("Deploying Macon")
        self.robot.actionneurs.deployMacon()

    def update(self):
        if self.robot.actionneurs.Servo_IO.isMoving(Actionneur.BrasPince.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.VerrouPince.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AimantHautGauche.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AimantHautDroit.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AimantBasGauche.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AimantBasDroit.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.PlancheGauche.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.PlancheDroit.value):
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

class waitCalibration(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name=f"Waiting Calibration")
        self.bb, self.robot, self.world = get_bb_robot(self)
    
    def initialise(self) -> None:
        self.robot.actionneurs.calibrateLift()

    def update(self):
        if self.robot.actionneurs.liftCalibrated:
            print("Calibrated !")
            return py_trees.common.Status.SUCCESS
        print("Waiting Calibration")
        return py_trees.common.Status.FAILURE


class AlignPlanches(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="Align Planches")
        self.bb, self.robot, self.world = get_bb_robot(self)

    def update(self):
        a, d, _ = self.robot.vl53_planches2()
        if a < radians(-3):
            # rotate left
            self.robot.locomotion.set_speed(Speed(0, 0, -0.1))
        elif a > radians(3):
            # rotate right
            self.robot.locomotion.set_speed(Speed(0, 0, 0.1))
        else:
            print("Planches aligned")
            self.robot.locomotion.set_speed(Speed(0, 0, 0))
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
        


class AlignConserves(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="Align Conserves")
        self.bb, self.robot, self.world = get_bb_robot(self)

    def update(self):
        x,y = self.robot.cameras.cam_cons()
        
        if y is not None:
            if abs(y) < 8: # mm, à régler
                print("Je suis aligné !!!")
                self.robot.move(0, 0, 0)
                # self.robot.buzz(ord('G'))
                return py_trees.common.Status.RUNNING # remplacer par succes
            else:
                dir = -y/abs(y)
                print(f"Bouge de {y} direction {'droite' if dir<0 else 'gauche'}\n")
                self.robot.move(abs(y), dir*radians(90), 80)
        else:
            #On voit rien du tout
            print("Je vois rien")
            self.robot.move(0, 0, 0)
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING
    
        # ix, d = self.robot.detect_best_conserve(Actionneur.AimantBasGauche)
        # ix2, d = self.robot.detect_best_conserve(Actionneur.AimantBasDroit)
        # if (ix < 3 and ix2 > 4) or (ix > 4 and ix2 < 3):
        #     return py_trees.common.Status.FAILURE

        # if ix < 3 or ix2 < 3:
        #     # move left
        #     self.robot.locomotion.set_speed(Speed(0, 15, 0))
        # elif ix > 4 or ix2 > 4:
        #     # move right
        #     self.robot.locomotion.set_speed(Speed(0, -15, 0))
        # else:
        #     print("Conserves aligned")
        #     return py_trees.common.Status.SUCCESS
        # return py_trees.common.Status.RUNNING


class Aligne_conserve_seul(py_trees.behaviour.Behaviour):
    def __init__(self, actionneur:Actionneur):
        super().__init__(name="Align Conserve Seul")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.actionneur = actionneur
        if self.actionneur == Actionneur.AimantBasGauche:
            self.sign = 1
        else:
            self.sign = -1
    def update(self):
        ix, d = self.robot.detect_one_conserve(self.actionneur)
        if d > 200:
            self.robot.locomotion.set_speed(Speed(0, 0, self.sign *0.5))  
        else:
            if ix < 3:
                # move left
                self.robot.locomotion.set_speed(Speed(0, 0, 0.2))
            elif ix > 4:
                # move right
                self.robot.locomotion.set_speed(Speed(0, 0, -0.2))
            else:
                print("Conserve aligned")
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING



class AvanceConserve(py_trees.behaviour.Behaviour):
    def __init__(self, actionneur:Actionneur):
        super().__init__(name="Avance Conserve")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.actionneur = actionneur
    
    def initialise(self):
        _, d = self.robot.detect_one_conserve(self.actionneur)
        self.robot.move(d-40, 0, 200)

    def update(self):
        if self.robot.hasReachedTarget():
            print("Avance Conserve done")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
    

class AvancePlanches(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="Avance Planches")
        self.bb, self.robot, self.world = get_bb_robot(self)
    
    def initialise(self):
        _, d, _ = self.robot.vl53_planches2()
        self.robot.move(d-40, 0, 100)

    def update(self):
        if self.robot.hasReachedTarget():
            print("Avance Planches done")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class LiftBanderole(py_trees.behaviour.Behaviour):
    def __init__(self,up:bool):
        super().__init__(name="Lift Banderole")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.up = up
    
    def initialise(self):
        self.robot.actionneurs.liftBanderole(self.up)

    def update(self):
        if self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AscenseurBanderolle.value):
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS
        
        
class Deplace_toi (py_trees.behaviour.Behaviour):
    def __init__(self, distance, direction_deg, vitesse):
        super().__init__(name="Deplace toi un peu en reculant")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.distance = distance
        self.direction = direction_deg
        self.vitesse = vitesse
    
    def initialise(self):
        print("Deplace toi un peu en reculant")
        self.robot.move(self.distance, radians(self.direction), self.vitesse)

    def update(self):
        if self.robot.hasReachedTarget():
            print("Deplacement fini")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


