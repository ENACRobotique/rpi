#!/usr/bin/env python3
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.composites import Selector, Sequence
import sys
sys.path.append("../..")
from robot import Robot
from world import World
from IO.IO_BT import * 
from bt_essentials import*
from planner import Planner, Action
from actions import *
import ecal.core.core as ecal_core


# === Boucle principale ===
if __name__ == "__main__":
    r = Robot()
    w = World()
    blackboard = py_trees.blackboard.Client(name="Foo Global")
    blackboard.register_key(key="robot", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="world", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="matchTime", access=py_trees.common.Access.WRITE)
    blackboard.robot = r
    blackboard.world = w
    blackboard.matchTime = 0

    planner = Planner(r, w)
    planner.add_action(MatchStartAction)
    planner.add_action(EndAction)
    planner.add_action(BanderoleAction)

    while ecal_core.ok():
        action = planner.plan()
        b = action.create_bt(r,w)
        mainBt = Selector(f"{b.name}", False, [EndMatch(10), b])
        
        bt = py_trees.trees.BehaviourTree(mainBt)
        bt.setup(timeout=15)
        print(f"Launching new action: {action.name}")
        while bt.root.status != py_trees.common.Status.SUCCESS and \
                bt.root.status != py_trees.common.Status.FAILURE:
            bt.tick()
            time.sleep(0.01)
        print(f"action {action.name} finished with status {bt.root.status.name}")
    
    ecal_core.finalize()
    r.actionneurs.Servo_IO.client.destroy()
    r.locomotion.stop()
