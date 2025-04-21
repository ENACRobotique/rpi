import py_trees
import sys
sys.path.append("../..")
from sw.IO.IO_BT import * 

class WaitSeconds(py_trees.behaviour.Behaviour):
    """Non blocking waiting"""
    def __init__(self, delay:float|int):
        super().__init__(name=f"Waiting {delay} second")
        self.delay = delay
        self.startingTime = -1

    def update(self):
        if self.startingTime == -1:
            print(f"Waiting {self.delay} second\n")
            self.startingTime = time.time() # init timer
        if abs(time.time()-self.startingTime)>= self.delay :
            print("Finished waiting") 
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

def joystick_bt(jerome: IO_Manager):
    """Joystick will use this"""
    
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

    root = py_trees.composites.Sequence("Root", True)
    root.add_children([
        waitCalibration(jerome),
        DeployMacon(jerome),
        WaitSeconds(2),
        ramasseGradin])
    return root

