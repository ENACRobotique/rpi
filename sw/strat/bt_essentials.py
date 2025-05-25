import py_trees
import sys
import time
sys.path.append("../..")
from robot import Robot, Pos, Tirette
from world import World

def get_bb_robot(behavior: py_trees.behaviour.Behaviour) -> tuple[py_trees.blackboard.Client, Robot, World]:
    bb = behavior.attach_blackboard_client(name="Foo Global")
    bb.register_key(key="robot", access=py_trees.common.Access.WRITE)
    bb.register_key(key="world", access=py_trees.common.Access.WRITE)
    robot: Robot = bb.robot
    world: World = bb.world
    return bb, robot, world


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
    def __init__(self, matchDuration):
        super().__init__(name=f"Match End ?")
        self.matchDuration=matchDuration
        self.MatchEnd = False
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.bb.register_key(key="matchTime", access=py_trees.common.Access.READ)
    
    def update(self):
        if self.bb.matchTime > 0:
            print(f"{abs(self.bb.matchTime-time.time())}")
            if abs(self.bb.matchTime-time.time()) >= self.matchDuration :
                print("Achievement Made! The End ?")
                self.robot.shuffle_play()
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
    
class Navigate(py_trees.behaviour.Behaviour):
    """TODO"""
    def __init__(self, dest, orientation):
        super().__init__(name=f"Navigating")
        self.bb = self.attach_blackboard_client(name="Foo Global")
        self.bb.register_key(key="robot", access=py_trees.common.Access.WRITE)
        self.robot: Robot = self.bb.robot
        self.dest = dest
        self.orientation = orientation
        


    def initialise(self):
        print("navigator init")
        self.robot.pathFinder(self.dest, self.orientation)
        self.robot.setTargetPos(self.robot.nav_pos[0])

    def update(self):
        if self.robot.hasReachedTarget():
            del self.robot.nav_pos[0]
            if self.robot.isNavDestReached():
                return py_trees.common.Status.SUCCESS
            else:
                self.robot.setTargetPos(self.robot.nav_pos[0])
            
        # Moving
        return py_trees.common.Status.RUNNING
    

class Move(py_trees.behaviour.Behaviour):
    """TODO"""
    def __init__(self, robot: Robot, distance, direction, speed):
        super().__init__(name=f"Navigating")
        self.robot = robot
        self.distance = distance
        self.direction = direction
        self.speed = speed

    def initialise(self):
        self.robot.move(self.distance, self.direction, self.speed)

    def update(self):
        if self.robot.hasReachedTarget():
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

class WaitMatchStart(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name=f"WaitMatchStart")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.bb.register_key(key="matchTime", access=py_trees.common.Access.WRITE)
        self.firstIN = False
        self.matchStarted = False

    def update(self):
        if self.robot.tirette == Tirette.IN:
            if not self.firstIN:
                print("Tirette in for first time")
                self.robot.buzz(ord('B'))
            self.firstIN = True
        
        if self.firstIN :
            if not self.matchStarted:
                if self.robot.ready_to_go():
                    print("Match Started !")
                    self.bb.matchTime = time.time()
                    self.robot.buzz(ord('E')+7)
                    self.matchStarted = True
        if self.matchStarted:    
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
