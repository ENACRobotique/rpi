from planner import Action
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.composites import Selector, Sequence
import sys
sys.path.append("../..")
from robot import Robot
from common import Speed
from world import World
from IO.IO_BT import LiftBanderole
from bt_essentials import MatchTimer, Navigate, WaitMatchStart
from bt_essentials import Deplace_toi, EndStrat, END_POS, Bouge, WaitSeconds, INTEREST
from typing import Callable
from dataclasses import dataclass
import time
from locomotion import Velocity
import subprocess
##########################
###  Action Banderole  ###
##########################

class BanderoleAction(Action):
    name = "Banderole"

    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        poserBanderolle = py_trees.composites.Sequence("Poser la banderolle", True)
        poserBanderolle.add_children([
            # GoTo zone banderole
            WaitSeconds(0.5),
            Bouge(Speed.from_dir(-120,100), 2),
            LiftBanderole(False),
            Bouge(Speed.from_dir(-120,-100), 3),
        ])
        return poserBanderolle
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        if world.banderole_deployed:
            return 0
        else:
            return 20
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        if status == py_trees.common.Status.SUCCESS:
            world.banderole_deployed = True
            robot.updateScore(20)


##########################
###     Action End     ###
##########################

class EndAction(Action):
    name = "End"
    
    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        return EndStrat()
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        if world.time_left() > 0:
            # tiny reward to select this one if no  other action is possible
            return 1
        else:
            # crazy high reward to be sure to be selected
            return 1e6


##########################
### Action Match Start ###
##########################

class MatchStartAction(Action):
    name = "MatchStart"

    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        match_start = py_trees.composites.Sequence("Poser la banderolle", True)
        match_start.add_children([
            LiftBanderole(True),
            WaitMatchStart(),
            LiftBanderole(True)
        ])
        return match_start
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        # match not started yet
        if not world.match_started():
            return 1e6
        else:
            return 0
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        if status == py_trees.common.Status.SUCCESS:
            subprocess.Popen(["ecal_rec","-r","110","--activate"])


##########################
###   Action Go Home   ###
##########################

class GoHomeAction(Action):
    name = "GoHome"

    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        _pos, nav_pt = END_POS[robot.color][robot.strat]
        def nana(_):
            return nav_pt
        return Navigate(nana)
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        if not world.match_started() or world.backInZone:
            # match not started or already back home
            return 0
        
        end_pos, _nav_pt = END_POS[robot.color][robot.strat]
        estimated_time = robot.pos.distance(end_pos) / robot.locomotion.speed + 5
        if world.time_left() < estimated_time:
            # rush to home, high reward
            return 1000
        else:
            return 2
    
    @staticmethod
    def start_cb(robot: Robot, world: World) -> None:
        robot.locomotion.select_velocity(Velocity.NORMAL)
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        if status == py_trees.common.Status.SUCCESS:
            world.backInZone = True
            # On lève la pince et on l'avance le plus possible pour avoir la projection verticale dans la zone de fin
            robot.actionneurs.deployPince(True)
            robot.actionneurs.lockPlanche(False)
            robot.updateScore(10)

##########################
###   Action PoussePousse   ###
##########################

class PoussePousse(Action):
    name = "poussepousse"

    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        nav_pt = INTEREST[robot.color][robot.strat]
        def nana(_):
            return nav_pt
        simple_pousse = py_trees.composites.Sequence("poussepousse", True)
        simple_pousse.add_children([
            Navigate(nana),
            Bouge(Speed(100,0,0), 5),
            Bouge(Speed(-100,0,0), 2),
        ])
        return simple_pousse
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        gradin = INTEREST[robot.color][robot.strat][0]
        if world.Gradin[gradin]:
            return 0
        else:
            return 5
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        if status == py_trees.common.Status.SUCCESS:
            gradin = INTEREST[robot.color][robot.strat][0]
            world.Gradin[gradin] = True
            robot.updateScore(4)
