import py_trees
import sys
import time
sys.path.append("../..")
from robot import Robot

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

class EndMatch(py_trees.behaviour.Behaviour):
    """TODO: \n 
    - update score\n
    - stop robot"""
    def __init__(self):
        super().__init__(name=f"Match End ?")
        self.MatchEnd = False
    
    def update(self):
        if self.MatchEnd:
            print("Achievement Made! The End ?")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
    
class Navigate(py_trees.behaviour.Behaviour):
    """TODO"""
    def __init__(self, robot: Robot, dest, orientation):
        super().__init__(name=f"Navigating")
        self.robot = robot
        self.dest = dest
        self.orientation = orientation

    def initialise(self) :
        self.robot.pathFinder(self.dest, self.orientation)

    def update(self):
        self.robot.setTargetPos(self.robot.nav_pos[0])
        if self.robot.isNavDestReached():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
 
class Evitement(py_trees.behaviour.Behaviour):
    """TODO: ALL"""
    def __init__(self):
        super().__init__(name=f"Evitement")