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

    def initialise(self):
        self.robot.pathFinder(self.dest, self.orientation)

    def update(self):
        self.robot.setTargetPos(self.robot.nav_pos[0])
        if self.robot.hasReachedTarget():
            del self.robot.nav_pos[0]
            if self.robot.isNavDestReached():
                # print(f"{self.dest} reached !\n")
                return py_trees.common.Status.SUCCESS
        # Moving
        return py_trees.common.Status.RUNNING
 
class Evitement(py_trees.behaviour.Behaviour):
    """TODO:
    - evitement basique : on s'arrete DONE  !
    - evitement intermédiare : on recule 
    - evitement avancé : on countourne"""
    def __init__(self, robot:Robot):
        super().__init__(name=f"Evitement")
        self.robot = robot
        self.evitement = False

    def initialise(self):
        self.last_target = self.robot.last_target # on retient la dernière consigne à chaque appel. Tant que update renvoie FAILURE cette fonction sera apellée
        self.evitement = False

    def update(self):
        if self.robot.obstacle_in_way(self.last_target):# self.last_target):
            # print("Avoiding")
            self.robot.setTargetPos(self.robot.pos)
            self.evitement = True
            return py_trees.common.Status.RUNNING # adversaire detecté, évitement en cours !
        # print("Nothing to avoid")
        return py_trees.common.Status.FAILURE # pas d'advresaire detecté 
    
    def terminate(self, new_status: py_trees.common.Status):
        if new_status == py_trees.common.Status.FAILURE: ## si on a une state INVALID on préfère ne rien faire (pour l'instant)
            if self.evitement: # on verifie qu'on a evité quelque chose pour ne pas perturber le robot
                self.robot.setTargetPos(self.last_target) # le robot repart
                print(self.last_target)