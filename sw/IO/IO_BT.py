import py_trees
import sys
sys.path.append("../..")
from sw.IO.actionneurs import*

##### Behavior tree pour l'automatisation #####

class LiftPlanche(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager, up:bool):
        pos = "DOWN"
        if up:
            pos = "UP"
        super().__init__(name=f"Lift {pos} planche")
        self.manager = manager
        self.up = up
        self.done = False
        self.pos = pos

    def update(self):
        if not self.done:
            self.done= True
            print(f"Lift {self.pos} planche")
            self.manager.liftPlanches(self.up)
        return py_trees.common.Status.SUCCESS


class LiftConserve(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager, up:bool):
        pos = "DOWN"
        if up:
            pos = "UP"
        super().__init__(name=f"Lift {pos} Conserve")
        self.manager = manager
        self.up = up
        self.done = False
        self.pos = pos

    def update(self):
        if not self.done:
            self.done= True
            print(f"Lift {self.pos} Conserve")
            self.manager.liftConserve(self.up)
        return py_trees.common.Status.SUCCESS
    
class MoveRentreur(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager, position:bool):
        pos = "OUTSIDE"
        if position:
            pos = "INSIDE"
        super().__init__(name=f"Rentreur going {pos}")
        self.manager = manager
        self.position = position
        self.done = False
        self.pos = pos

    def update(self):
        if not self.done:
            self.done= True
            print(f"Rentreur going {self.pos}")
            self.manager.moveRentreur(self.position)
        return py_trees.common.Status.SUCCESS


class GrabHighConserve(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager, grab : bool):
        status = "Dropping"
        if grab:
            status = "Grabbing"
        super().__init__(name=f"{status} High Conserve")
        self.manager = manager
        self.grab = grab
        self.done = False
        self.status= status

    def update(self):
        if not self.done:
            self.done= True
            print(f"{self.status} High Conserve")
            self.manager.grabHighConserve(self.grab)
        return py_trees.common.Status.SUCCESS

class GrabLowConserve(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager, grab : bool):
        status = "Dropping"
        if grab:
            status = "Grabbing"
        super().__init__(name=f"{status} Low Conserve")
        self.manager = manager
        self.grab = grab
        self.done = False
        self.status = status

    def update(self):
        if not self.done:
            self.done= True
            print(f"{self.status} Low Conserve")
            self.manager.grabLowConserve(self.grab)
        return py_trees.common.Status.SUCCESS
    
class LockPlanche(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager, lock : bool):
        status = "Unlocking"
        if lock:
            status = "Locking"
        super().__init__(name=f"{status} Upper Planche")
        self.manager = manager
        self.lock = lock
        self.done = False
        self.status = status

    def update(self):
        if not self.done:
            self.done= True
            print(f"{self.status} Upper Planche")
            self.manager.lockPlanche(self.lock)
        return py_trees.common.Status.SUCCESS
    
class DeployMacon(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager):
        super().__init__(name=f"Deploying Macon")
        self.manager = manager
        self.deployed = False

    def update(self):
        if not self.deployed:
            print("Deploying Macon")
            self.manager.deployMacon()
            self.deployed = True
        return py_trees.common.Status.SUCCESS

class waitCalibration(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager):
        super().__init__(name=f"Waiting Calibration")
        self.manager = manager
        
    def update(self):
        if self.manager.liftCalibrated:
            print("Calibrated !")
            return py_trees.common.Status.SUCCESS
        print("Waiting Calibration")
        return py_trees.common.Status.FAILURE