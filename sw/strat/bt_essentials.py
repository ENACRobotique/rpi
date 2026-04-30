import py_trees
import sys
import time
import numpy as np
sys.path.append("../..")
from robot import Robot, Pos, Speed, Tirette, Team, Strat
from world import World
from math import radians
from collections.abc import Callable

START_POS = {
    Team.JAUNE: {
        Strat.Basique: ('NidJ',np.pi/2)
        # Strat.Audacieuse: ('secureJ', -pi/2)
    },
    Team.BLEU: {
        Strat.Basique: ('NidB',np.pi/2)
        # Strat.Audacieuse: ('secureB', -pi/2)
    }
}
END_POS = {
    Team.JAUNE: {
        Strat.Basique: ('NidJ',-np.pi/2)
        # Strat.Audacieuse: 'midJ'
    },
    Team.BLEU: {
        Strat.Basique: ('NidB',-np.pi/2)
        # Strat.Audacieuse: 'midB'
    }
}

THERMO_POS = {
    Team.JAUNE: {
        Strat.Basique: ('ThermoJ',0)
    },
    Team.BLEU: {
        Strat.Basique: ('ThermoB',np.pi)
    }
}

CAISSETHERMO_POS = {
    Team.JAUNE: {
        Strat.Basique: ('NoixJSW',np.pi/2)
    },
    Team.BLEU: {
        Strat.Basique: ('NoixBSE',np.pi/2)
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

class WaitUntil(py_trees.behaviour.Behaviour):
    """Non blocking waiting"""
    def __init__(self, date:float|int,dateStart:float|int):
        super().__init__(name=f"Waiting {date} second")
        self.date = date
        self.dateStart = dateStart

    def update(self):
        if abs(time.time()-self.dateStart)>= self.date :
            print("Finished waiting") 
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class EndStrat(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name=f"Force strat to end")
        self.bb, self.robot, self.world = get_bb_robot(self)
    
    def initialise(self):
        self.robot.set_speed(Speed(0, 0, 0))

    def update(self):
        print("Strat terminée !")
        # play tune ?
        # TODO: do this in a non blocking manner
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
                #print("Achievement Made! The End ?")
                self.robot.set_speed(Speed(0, 0, 0))
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
        self.robot.resetPosOnEkf()
        print("Navigation go !")
        if not self.robot.folowingPath:
            self.robot.pathFinder(self.dest, self.orientation)
            self.robot.setTargetPos(self.robot.nav_pos[self.nav_id])

    def update(self):
        if self.robot.obstacle_in_way(self.robot.nav_pos[self.nav_id]):
            if not self.avoiding:
                self.robot.log("Obstacle detected, stopping.")
                self.robot.set_speed(Speed(0, 0, 0))
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
                    if self.robot.closeToNavPoint(self.nav_id) and self.robot.hasReachedTarget() and self.nav_id < len(self.robot.nav_pos)-1:
                        self.nav_id+=1
                        self.robot.setTargetPos(self.robot.nav_pos[self.nav_id])
                        self.robot.log(f"Navigation :{self.robot.nav_pos[self.nav_id]} \n")
        return py_trees.common.Status.RUNNING
    

class Move(py_trees.behaviour.Behaviour):
    """TODO"""
    def __init__(self, robot: Robot, distance, direction, speed):
        super().__init__(name=f"Move")
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
    def __init__(self, position_target:Pos | Callable[[Robot], Pos]):
        super().__init__(name=f"MoveTo")
        self.bb, self.robot,_ = get_bb_robot(self)
        self.position_target = position_target
        self.dernier_consigne_pos = None
        self.avoiding = False

    def initialise(self):
        if isinstance(self.position_target, Pos):
            self.dernier_consigne_pos = self.position_target
        else :
             self.dernier_consigne_pos = self.position_target(self.robot)
        self.robot.setTargetPos(self.dernier_consigne_pos)


    def update(self):
        if self.robot.obstacle_in_way(self.dernier_consigne_pos):
             if not self.avoiding:
                self.robot.log("Obstacle detected, stopping.")
                self.robot.set_speed(Speed(0, 0, 0))
                #self.robot.setTargetPos(self.robot.pos) #ARRETER ROBOT
                print(f"Robot stop ici: {self.robot.pos}")
                self.avoiding = True
        else:   # pas d'obstacle
            if self.avoiding:
                # resume movement after obstacle avoidance
                self.robot.log("No obstacle, resuming movement")
                self.robot.setTargetPos(self.dernier_consigne_pos)
                self.avoiding = False
            else:
                if self.robot.hasReachedTarget():
                    return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


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
                self.last_time = time.time()
        if self.matchStarted:
            # last check
            desired_pos = self.robot.dest_to_pos(START_POS[self.color][self.robot.strat])
            if self.robot.pos.distance(desired_pos) > 200:
                self.robot.resetPos(desired_pos,0)
            return py_trees.common.Status.SUCCESS
        if self.robot.tirette == Tirette.IN:
            if not self.firstIN:
                self.robot.log("Tirette in for first time")
            self.firstIN = True
        
        if self.firstIN :
            if not self.matchStarted:
                if self.robot.ready_to_go(): 
                    self.robot.log("Match Started !")
                    self.world.matchStartTime = time.time()
                    self.matchStarted = True
        if self.color != self.robot.color:
            self.color = self.robot.color
            print("reset posssssssssss")
            self.robot.resetPos(self.robot.dest_to_pos(START_POS[self.color][self.robot.strat]))
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
        self.robot.resetPos(self.position,0)
        print("Recalage")

    def update(self):
        if self.done:
            return py_trees.common.Status.SUCCESS
        if time.time() - self.init_time > self.timeout:
            self.done = True
            self.robot.log(f"Pos reseted to : {self.position}")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
