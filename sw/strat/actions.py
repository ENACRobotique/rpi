from planner import Action
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.composites import Selector, Sequence
import sys
sys.path.append("../..")
from robot import Robot
from world import World
from IO.IO_BT import Deplace_toi, LiftBanderole
from bt_essentials import EndMatch, Navigate, WaitMatchStart
from typing import Callable
from dataclasses import dataclass
import time


##########################
#### Action Banderole ####
##########################

class BanderoleAction(Action):
    name = "Banderole"

    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        poserBanderolle = py_trees.composites.Sequence("Poser la banderolle", True)
        poserBanderolle.add_children([
            # GoTo zone banderole
            Deplace_toi(80,-120-4,50),
            LiftBanderole(False),
            Deplace_toi(-250,-120-4,100)
        ])
        return poserBanderolle
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        if world.banderole_deployed:
            return 0
        else:
            return 20

##########################
####    Action End    ####
##########################

class EndAction(Action):
    name = "End"
    
    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        return EndMatch(10)
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        if world.matchStartTime is not None and \
            time.time() - world.matchStartTime < world.MATCH_DURATION:
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
        return WaitMatchStart()
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        # match not started yet
        if world.matchStartTime < 0:
            return 1e6
        else:
            return 0
