import py_trees
import sys
sys.path.append("../..")
from sw.actionneurs import*

##### Behavior tree pour l'automatisation #####

class LiftPlanche(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager, up:bool):
        pos = "DOWN"
        if up:
            pos = "UP"
        super().__init__(name=f"Lift {pos} planche")
        self.manager = manager
        self.manager.liftPlanches(up)

    def update(self):
        py_trees.common.Status.SUCCESS


class LiftConserve(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager, up:bool):
        pos = "DOWN"
        if up:
            pos = "UP"
        super().__init__(name=f"Lift {pos} Conserve")
        self.manager = manager
        self.manager.liftConserve(up)

    def update(self):
        return py_trees.common.Status.SUCCESS
    
class MoveRentreur(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager, position:bool):
        pos = "OUTSIDE"
        if position:
            pos = "INSIDE"
        super().__init__(name=f"Rentreur going {pos}")
        self.manager = manager
        self.manager.moveRentreur(position)

    def update(self):
        return py_trees.common.Status.SUCCESS


class GrabHighConserve(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager, grab : bool):
        status = "Dropping"
        if grab:
            status = "Grabbing"
        super().__init__(name=f"{status} Rentreur Conserve")
        self.manager = manager
        self.grab = grab

    def update(self):
        self.manager.grabHighConserve(self.grab)
        return py_trees.common.Status.SUCCESS

class GrabLowConserve(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager, grab : bool):
        status = "Dropping"
        if grab:
            status = "Grabbing"
        super().__init__(name=f"{status} Rentreur Conserve")
        self.manager = manager
        self.grab = grab

    def update(self):
        self.manager.grabLowConserve(self.grab)
        return py_trees.common.Status.SUCCESS

class WaitSeconds(py_trees.behaviour.Behaviour):
    def __init__(self, delay:float|int):
        super().__init__(name=f"Waiting {delay} second")
        self.delay = delay
        self.startingTime = time.time()

    def update(self):
        if abs(time.time()-self.startingTime)>= self.delay : 
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
    
class LockPlanche(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager, lock : bool):
        status = "Unlocking"
        if lock:
            status = "Locking"
        super().__init__(name=f"{status} Upper Planche")
        self.manager = manager
        self.lock = lock

    def update(self):
        self.manager.lockPlanche(self.lock)
        return py_trees.common.Status.SUCCESS
    
class DeployMacon(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager):
        super().__init__(name=f"Deploying Pince")
        self.manager = manager
        self.manager.deployMacon()
        print("Deploying Macon")

    def update(self):
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

def test_bt(jerome :IO_Manager):
    root = py_trees.composites.Sequence("Root", False)
    
    ramasseGradin = py_trees.composites.Sequence("Rammasser un Gradin", True)
    ramasseGradin.add_children([
        LiftPlanche(jerome, UP),
        WaitSeconds(0.5),
        GrabHighConserve(jerome, False),
        GrabLowConserve(jerome, True),
        WaitSeconds(0.3),
        LiftConserve(jerome, UP),
        WaitSeconds(2),
        MoveRentreur(jerome, OUTSIDE),
        WaitSeconds(0.1),
        GrabHighConserve(jerome, True),
        WaitSeconds(1.5),
        GrabLowConserve(jerome, False),
        WaitSeconds(0.25),
        LockPlanche(jerome, True),
        LiftConserve(jerome, DOWN),
        WaitSeconds(0.5),

        GrabLowConserve(jerome, True),
        WaitSeconds(0.7),

        MoveRentreur(jerome, INSIDE),
        WaitSeconds(0.3),
        GrabLowConserve(jerome, False)
    ])

    construitGradin = py_trees.composites.Sequence("Construire un Gradin", True)
    construitGradin.add_children([
            LiftPlanche(jerome,DOWN),
            WaitSeconds(2.5),
            MoveRentreur(jerome, OUTSIDE),
            WaitSeconds(1.8),
            GrabHighConserve(jerome, False),
            WaitSeconds(0.3),
            MoveRentreur(jerome, INSIDE),
            WaitSeconds(0.5),
            LockPlanche(jerome, False),
            WaitSeconds(0.5),
            GrabLowConserve(jerome, False) 
    ])

    root.add_children([waitCalibration(jerome), DeployMacon(jerome), ramasseGradin])
    return root

