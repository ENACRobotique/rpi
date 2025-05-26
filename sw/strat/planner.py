#!/usr/bin/env python3
from py_trees.behaviour import Behaviour
import sys
sys.path.append("../..")
from robot import Robot
from world import World
from typing import Callable
from dataclasses import dataclass


class Action:
    name: str
    # a function that takes a Robot and the world state as arguments,
    # and returns a behavior tree to be executed
    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        raise Exception("Unimplemented!!!")
    
    # a function that takes a Robot and the world state as arguments,
    # a reward associated witht this action
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        raise Exception("Unimplemented!!!")



class Planner:
    def __init__(self, robot: Robot, world: World) -> None:
        self.actions: list[type[Action]] = []
        self.robot = robot
        self.world = world

    def add_action(self, action: type[Action]):
        self.actions.append(action)
    
    def plan(self) -> type[Action]:
        # return the action that yields the maximum reward
        action = max(self.actions, key=lambda a: a.reward(self.robot, self.world))
        return action
