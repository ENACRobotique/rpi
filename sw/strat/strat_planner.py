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
import musics


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
    planner.add_action(PoussePousse)
    planner.add_action(GoHomeAction)

    r.play_music(musics.smoke_on_the_water)

    while ecal_core.ok():
        action = planner.plan()
        b = action.create_bt(r,w)
        mainBt = Selector(f"{b.name}", False, [MatchTimer(), b])
        
        bt = py_trees.trees.BehaviourTree(mainBt)
        bt.setup(timeout=15)
        r.log(f"Launching new action: {action.name}")
        # print(f"Launching new action: {action.name}")
        action.start_cb(r, w)
        while bt.root.status != py_trees.common.Status.SUCCESS and \
                bt.root.status != py_trees.common.Status.FAILURE:
            bt.tick()
            time.sleep(0.01)
        action.end_cb(r, w, bt.root.status)
        r.log(f"action {action.name} finished with status {bt.root.status.name}")
        # print(f"action {action.name} finished with status {bt.root.status.name}")
    
    ecal_core.finalize()
    r.actionneurs.Servo_IO.client.destroy()
    r.locomotion.stop()
