from planner import Action
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.composites import Selector, Sequence
import sys
sys.path.append("../..")
from robot import Robot
from world import World
from IO.IO_BT import Deplace_toi, LiftBanderole
from bt_essentials import EndMatch, Navigate
from typing import Callable
from dataclasses import dataclass
import time


##########################
#### Action Banderole ####
##########################

def banderole_create(robot: Robot, world: World):
    poserBanderolle = py_trees.composites.Sequence("Poser la banderolle", True)
    poserBanderolle.add_children([
        # GoTo zone banderole
        Deplace_toi(80,-120-4,50),
        LiftBanderole(False),
        Deplace_toi(-250,-120-4,100)
    ])
    return poserBanderolle

def banderole_reward(robot: Robot, world: World):
    if world.banderole_deployed:
        return 0
    else:
        return 20

action_banderolle = Action("Banderolle", banderole_create, banderole_reward)


##########################
####    Action End    ####
##########################

def end_create(robot, world):
    return EndMatch(10)

def end_reward(robot: Robot, world: World):
    if robot.tempsDebutMatch is not None and \
        time.time() - robot.tempsDebutMatch < world.MATCH_DURATION:
        # tiny reward to select this one if no  other action is possible
        return 1
    else:
        # crazy high reward to be sure to be selected
        return 1000000

action_end_match = Action("MatchEnd", end_create, end_reward)
