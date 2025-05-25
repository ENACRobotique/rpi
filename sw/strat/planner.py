#!/usr/bin/env python3
from py_trees.behaviour import Behaviour
import sys
sys.path.append("../..")
from robot import Robot
from world import World
from typing import Callable
from dataclasses import dataclass


@dataclass
class Action:
    name: str
    # a function that takes a Robot and the world state as arguments,
    # and returns a behavior tree to be executed
    create_bt: Callable[[Robot, World], Behaviour]
    # a function that takes a Robot and the world state as arguments,
    # a reward associated witht this action
    reward: Callable[[Robot, World], int]



class Planner:
    def __init__(self, robot: Robot, world: World) -> None:
        self.actions: list[Action] = []
        self.robot = robot
        self.world = world

    def add_action(self, action: Action):
        self.actions.append(action)
    
    def plan(self) -> Action:
        # return the action that yields the maximum reward
        action = max(self.actions, key=lambda a: a.reward(self.robot, self.world))
        return action


