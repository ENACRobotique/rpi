from fsm import State, FSM
from state_essentials import *
import sys
sys.path.append("../")
from robot import Robot, Pos,Team
import time
from math import pi


class InitState(State):
    def enter(self, prev_state: State | None):
        print("Let's get it started in here !")
        print(f"strat name: {self.globals['strat_name']}")
        self.start_time = time.time()
    
    def loop(self):
        # tester si tirette tirée
        #self.robot.resetPosFromNav("basB")
        if time.time() - self.start_time > 1:
            
            args = {}
            if self.robot.color == Team.BLEU:
                self.robot.resetPos(Pos(225, 225, pi/2))
                args["panos"] =  ["p1", "p2", "p3", "p4", "p5", "p6"]
                args["pano_angle"] = 0
            else:
                args["panos"] = ["p9", "p8", "p7", "p6", "p5", "p4"]
                args["pano_angle"] = 180
            
            self.robot.pano_angle = args["pano_angle"]
            
            # args["destination"] = 'p1'
            # args["next_state"] = PanosState(self.robot, self.globals, args)
            
            return PanosState(self.robot, self.globals, args)






if __name__ == "__main__":
    robot = Robot()
    robot.initNav()
    globals = {"strat_name": "pano strat"}
    fsm = FSM(robot, InitState,globals)
    fsm.run()

