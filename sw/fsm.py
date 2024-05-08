from __future__ import annotations
from typing import Type
from typing import Optional, Generator
import sys
sys.path.append("../")
from robot import Robot
import time

MATCH_TIMEOUT = 88

class State:
    def __init__(self, robot: Robot, globals: dict, args: dict) -> None:
        self.robot = robot
        self.globals = globals
        self.args = args

    def enter(self, prev_state: State | None):
        #name = self.__class__.__name__
        #raise NotImplementedError(f'"enter" not implemented "{name}"!')
        ...
    
    def leave(self, next_state: State):
        #name = self.__class__.__name__
        #raise NotImplementedError(f'"leave" not implemented "{name}"!')
        ...
    
    def loop(self) -> Generator[State | None, None, None]:
        yield None
    
    def on_obstacle(self):
        last_target = self.robot.last_target
        self.robot.setTargetPos(self.robot.pos)
        while self.robot.obstacle_in_way(last_target):
            time.sleep(0.1)
        if self.robot.tempsDebutMatch is not None and time.time() - self.robot.tempsDebutMatch < MATCH_TIMEOUT:
            self.robot.setTargetPos(last_target)
            print("no more obstacles, resuming move")




class FSM:
    def __init__(self, robot: Robot, initStateCls: Type[State], end_state_cls:Type[State], globals={}, dt=0.05) -> None:
        self.robot = robot
        self.globals = globals
        self.current_state = initStateCls(robot, globals, {})
        self.dt = dt
        self.end_state_cls = end_state_cls

    def run(self):
        self.current_state.enter(None)
        print("\nState : ",self.current_state.__class__.__name__)
        while True:
            for next_state in self.current_state.loop():
                if self.robot.tempsDebutMatch is not None and time.time() - self.robot.tempsDebutMatch > MATCH_TIMEOUT:
                    print("\nTIMEOUT FIN DE MATCH\n")
                    if self.current_state.__class__ != self.end_state_cls:
                        print("fin du match!")
                        next_state = self.end_state_cls(self.robot, self.globals, {})
                if self.robot.obstacle_in_way(self.robot.last_target):
                    print("on obstacle")
                    self.current_state.on_obstacle()
                if next_state is not None:
                    self.current_state.leave(next_state)
                    print("\nState : ",next_state.__class__.__name__)
                    next_state.enter(self.current_state)
                    self.current_state = next_state
                    break
                time.sleep(self.dt)



# if __name__ == "__main__":
#     robot = Robot()
#     globals = {"strat_name": "test strat"}
#     fsm = FSM(robot, TestState, globals)
