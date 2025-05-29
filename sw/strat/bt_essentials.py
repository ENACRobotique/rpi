import py_trees
import sys
import time
sys.path.append("../..")
from robot import Robot, Pos, Speed, Tirette, Team, Strat
from world import World
from math import radians
from collections.abc import Callable

START_POS = {
    Team.JAUNE: {
        Strat.Basique: Pos(1200, 250, radians(30)),
        # Strat.Audacieuse: ('secureJ', -pi/2)
    },
    Team.BLEU: {
        Strat.Basique: Pos(1800, 250, radians(30)),
        # Strat.Audacieuse: ('secureB', -pi/2)
    }
}
END_POS = {
    Team.JAUNE: {
        Strat.Basique: [Pos(200, 1400, radians(90)),('finJ',radians(90))],
        # Strat.Audacieuse: 'midJ'
    },
    Team.BLEU: {
        Strat.Basique: [Pos(2800, 1400, radians(90)),('finB',radians(90))],
        # Strat.Audacieuse: 'midB'
    }
}
INTEREST = {
    Team.JAUNE:{
        Strat.Basique: ('G3', radians(-90)),
    },
    Team.BLEU:{
        Strat.Basique: ('G4', radians(-90)),
    }
}
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

class EndStrat(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name=f"Force strat to end")
        self.bb, self.robot, self.world = get_bb_robot(self)
    
    def initialise(self):
        self.robot.locomotion.set_speed(Speed(0, 0, 0))

    def update(self):
        print("Strat terminée !")
        # play tune ?
        # TODO: do this in a non blocking manner
        self.robot.shuffle_play()
        return py_trees.common.Status.RUNNING

class MatchTimer(py_trees.behaviour.Behaviour):
    """TODO: \n 
    - update score\n
    - stop robot"""
    def __init__(self):
        super().__init__(name=f"Match End ?")
        self.MatchEnd = False
        self.bb, self.robot, self.world = get_bb_robot(self)
    
    def update(self):
        if self.world.matchStartTime > 0:
            if abs(self.world.matchStartTime-time.time()) >= self.world.MATCH_DURATION:
                print("Achievement Made! The End ?")
                self.robot.locomotion.set_speed(Speed(0, 0, 0))
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
    
class Navigate(py_trees.behaviour.Behaviour):
    """TODO"""
    def __init__(self, nav_cb:Callable[[Robot], tuple]):
        super().__init__(name=f"Navigating")
        self.bb, self.robot, self.world = get_bb_robot(self)  # ensure blackboard and robot are initialized
        self.nav_cb = nav_cb
        self.nav_id = 0
        self.avoiding = False
        # TODO ajouter un timeout pour l'évitement ?

    def initialise(self):
        self.dest, self.orientation = self.nav_cb(self.robot)
        print("Navigation go !")
        if not self.robot.folowingPath:
            self.robot.pathFinder(self.dest, self.orientation)
            self.robot.setTargetPos(self.robot.nav_pos[self.nav_id])

    def update(self):
        if self.robot.obstacle_in_way(self.robot.nav_pos[self.nav_id]):
            if not self.avoiding:
                self.robot.log("Obstacle detected, stopping.")
                self.robot.locomotion.set_speed(Speed(0, 0, 0))
                self.avoiding = True
        else:   # pas d'obstacle
            if self.avoiding:
                # resume movement after obstacle avoidance
                self.robot.log("No obstacle, resuming movement")
                self.robot.setTargetPos(self.robot.nav_pos[self.nav_id])
                self.avoiding = False
            else:
                # normal case, no obstacle

                if self.robot.isNavDestReached():
                    print("Navigation end !")
                    return py_trees.common.Status.SUCCESS
                else:
                    if self.robot.closeToNavPoint(self.nav_id) and self.nav_id < len(self.robot.nav_pos)-1:
                        self.nav_id+=1
                        self.robot.setTargetPos(self.robot.nav_pos[self.nav_id])
                        self.robot.log(f"Navigation :{self.robot.nav_pos[self.nav_id]} \n")
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
        print("[Move] TODO EVITEMENT!!!!!!!!!!!!!!!!!!!!!!!!")
        if self.robot.hasReachedTarget():
            return py_trees.common.Status.SUCCESS
        # Moving
        return py_trees.common.Status.RUNNING
    
class MoveTo(py_trees.behaviour.Behaviour):
    def __init__(self, position_target:Pos):
        super().__init__(name=f"MoveTo")
        self.bb, self.robot = get_bb_robot(self)
        self.position_target = position_target

    def initialise(self):
        self.robot.setTargetPos(self.position_target)

    def update(self):
        print("[MoveTo] TODO EVITEMENT!!!!!!!!!!!!!!!!!!!!!!!!")
        if self.robot.hasReachedTarget():
            return py_trees.common.Status.SUCCESS
        self.robot.setTargetPos(self.position_target)
        # Moving
        return py_trees.common.Status.RUNNING

class Deplace_toi (py_trees.behaviour.Behaviour):
    def __init__(self, distance, direction_deg, vitesse):
        super().__init__(name="Deplace toi un peu en reculant")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.distance = distance
        self.direction = direction_deg
        self.vitesse = vitesse
        self.done = False
    def initialise(self):
        if self.done:
            return
        print("Deplace toi un peu en reculant")
        self.robot.move(self.distance, radians(self.direction), self.vitesse)

    def update(self):
        print("[Deplace_toi] TODO EVITEMENT!!!!!!!!!!!!!!!!!!!!!!!!")
        if self.done:
            return py_trees.common.Status.SUCCESS
        if self.robot.hasReachedTarget():
            print("Deplacement fini")
            self.done = True
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class Bouge (py_trees.behaviour.Behaviour):
    def __init__(self, vitesse: Speed, temps):
        super().__init__(name="Bouge")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.temps = temps
        self.vitesse = vitesse
        self.done = False
        self.start_time: float = 0
        self.pause_time = None

    def initialise(self):
        self.robot.locomotion.set_speed(self.vitesse,self.temps)
        self.start_time = time.time()
    
    def update(self):
        # robot stopped and not in pause
        if self.robot.locomotion.is_idle() and self.pause_time is None:
            print("Deplacement fini")
            self.done = True
            return py_trees.common.Status.SUCCESS
        if self.robot.obstacle_in_speed(self.vitesse):
            if self.pause_time is None:
                self.robot.log(f"Obstacle detected, stopping.   {self.vitesse}")
                self.robot.locomotion.set_speed(Speed(0, 0, 0), 10)
                self.pause_time = time.time()
        else:   # pas d'obstacle
            if self.pause_time is not None:
                self.robot.log("No obstacle, resuming movement")
                time_left = self.temps - (self.pause_time - self.start_time)
                self.robot.locomotion.set_speed(self.vitesse, time_left)
                self.pause_time = None
                self.temps = time_left
                self.start_time = time.time()
        return py_trees.common.Status.RUNNING

# class Evitement(py_trees.behaviour.Behaviour):
#     """TODO:
#     - evitement basique : on s'arrete DONE  !
#     - evitement intermédiare : on recule 
#     - evitement avancé : on countourne"""
#     def __init__(self):
#         super().__init__(name=f"Evitement")
#         self.bb, self.robot, self.world = get_bb_robot(self)
#         self.bb.register_key(key="matchTime", access=py_trees.common.Access.WRITE)
#         self.evitement = False

#     def initialise(self):
#         self.last_target = self.robot.last_target # on retient la dernière consigne à chaque appel. Tant que update renvoie FAILURE cette fonction sera apellée
#         self.evitement = False

#     def update(self):
#         if self.bb.matchTime == 0 :
#             return py_trees.common.Status.FAILURE
#         if self.robot.obstacle_in_way(self.last_target):
#             print("Avoiding")
#             # self.robot.setTargetPos(self.robot.pos)
#             self.robot.locomotion.set_speed(Speed(0, 0, 0))
#             # reculer ? :
#             # self.robot.locomotion.set_speed(Speed(-self.robot.speed.vx, -self.robot.speed.vy, 0), 0.5) 
#             self.evitement = True
#             return py_trees.common.Status.RUNNING # adversaire detecté, évitement en cours !
#         return py_trees.common.Status.FAILURE # pas d'advresaire detecté 
    
#     def terminate(self, new_status: py_trees.common.Status):
#         if new_status == py_trees.common.Status.FAILURE: ## si on a une state INVALID on préfère ne rien faire (pour l'instant)
#             if self.evitement: # on verifie qu'on a evité quelque chose pour ne pas perturber le robot
#                 # self.robot.setTargetPos(self.last_target) # le robot repart
#                 print(f"Resuming: {self.last_target}\n")

class WaitMatchStart(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name=f"WaitMatchStart")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.firstIN = False
        self.matchStarted = False
        self.color = self.robot.color
        self.last_time = time.time()
    
    def initialise(self):
        self.last_time = time.time()


    def update(self):
        if self.robot.color == Team.AUCUNE:
            if time.time() - self.last_time > 0.5:
                # Waiting for robot color to be set
                self.robot.buzz(ord('B')+7)
                self.last_time = time.time()
        if self.matchStarted:
            return py_trees.common.Status.SUCCESS
        if self.robot.tirette == Tirette.IN:
            if not self.firstIN:
                self.robot.log("Tirette in for first time")
                self.robot.buzz(ord('B'))
            self.firstIN = True
        
        if self.firstIN :
            if not self.matchStarted:
                if self.robot.ready_to_go():
                    self.robot.log("Match Started !")
                    self.world.matchStartTime = time.time()
                    self.robot.buzz(ord('E')+7)
                    self.matchStarted = True
        if self.color != self.robot.color:
            self.color = self.robot.color
            self.robot.resetPosNonBlocking(START_POS[self.color][self.robot.strat])
        return py_trees.common.Status.RUNNING


class Recalage(py_trees.behaviour.Behaviour):
    def __init__(self, pos_cb:Callable[[Robot], Pos], timeout=0.5):
        """
        pos_cb: a callback that take a Robot as parameter, and returns the recalage position
        timeout: max ACK time
        """
        super().__init__(name=f"Recalage")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.bb.register_key(key="pos_recalage", access=py_trees.common.Access.READ)
        self.position: Pos = None
        self.timeout = timeout
        self.pos_cb = pos_cb
        self.done = False

    def initialise(self):
        if self.done:
            return
        self.position = self.pos_cb(self.robot)
        self.init_time = time.time()
        self.robot.resetPosNonBlocking(self.position)
        print("Recalage")

    def update(self):
        if self.done:
            return py_trees.common.Status.SUCCESS
        if time.time() - self.init_time > self.timeout:
            self.done = True
            self.robot.log(f"Pos reseted to : {self.position}")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
