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
        self.bb, self.robot = get_bb_robot(self)
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
        self.bb, self.robot = get_bb_robot(self)
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
        self.bb, self.robot = get_bb_robot(self)
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
        self.bb, self.robot = get_bb_robot(self)
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
        self.bb, self.robot = get_bb_robot(self)
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
        self.bb, self.robot = get_bb_robot(self)
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
        self.bb, self.robot = get_bb_robot(self)
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
        self.bb, self.robot = get_bb_robot(self)
    
    def initialise(self) -> None:
        self.robot.actionneurs.calibrateLift()

    def update(self):
        if self.robot.actionneurs.liftCalibrated:
            print("Calibrated !")
            return py_trees.common.Status.SUCCESS
        print("Waiting Calibration")
        return py_trees.common.Status.FAILURE