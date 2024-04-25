#!/usr/bin/python3
from fsm import State, FSM
from state_essentials import *
import sys
sys.path.append("../")
from robot import Robot, Pos,Team, Tirette, Strat
import robot
import time
from math import pi

BLUE_SPOT = {'Safe': 'secureB', 'Ennemy' : 'midB' , 'Down' : 'basB'}
YELLOW_SPOT = {'Safe': 'secureJ', 'Ennemy' : 'midJ' , 'Down' : 'basJ'}

START_POS = {
    Team.JAUNE: {
        Strat.Basique: ('basJ', pi/2),
        Strat.Audacieuse: ('secureJ', -pi/2)
    },
    Team.BLEU: {
        Strat.Basique: ('basB', pi/2),
        Strat.Audacieuse: ('secureB', -pi/2)
    }
}

END_POS = {
    Team.JAUNE: {
        Strat.Basique: 'midJ',
        Strat.Audacieuse: 'midJ'
    },
    Team.BLEU: {
        Strat.Basique: 'midB',
        Strat.Audacieuse: 'midB'
    }
}

STRAT_DATA = {
    Team.JAUNE: {
        "panos": ["p9", "p8", "p7", "p6", "p5", "p4"],
        "pano_angle": 180
    },
    Team.BLEU: {
        "panos": ["p1", "p2", "p3", "p4", "p5", "p6"],
        "pano_angle": 0
    }
}


globals = {
    "strat_name": "pano strat",
    "end_pos": None,
    "data": None
}

class PreInit(State):
    def loop(self):
        if self.robot.tirette == Tirette.IN:
            return InitState(self.robot, self.globals, {})

class InitState(State):
    def enter(self, prev_state: State | None):
        print("Let's get it started in here !")
        print(f"strat name: {self.globals['strat_name']}")
        self.start_time = time.time()
    
    def loop(self):
        # tester si tirette tirée
        #self.robot.resetPosFromNav("basB")
        if self.robot.tirette == Tirette.OUT:
            print(f"start with color {self.robot.color} ans strat {self.robot.strat}!")
            self.globals["end_pos"] = END_POS[self.robot.color][self.robot.strat]
            self.globals["data"] = STRAT_DATA[self.robot.color]
            start_pos = START_POS[self.robot.color][self.robot.strat]
            w, theta = start_pos
            wx, wy = self.robot.nav.getCoords(w)
            rp = Pos(wx, wy, theta)
            while self.robot.pos.distance(rp) > 5:
                self.robot.resetPosFromNav(*start_pos)
                time.sleep(0.1)
            args = {
                "panos": self.globals["data"]["panos"],
                "pano_angle": self.globals["data"]["pano_angle"],
            }
            #args["panos"] =  STRAT_DATA[self.robot.color]["panos"].copy()
            #args["pano_angle"] =  STRAT_DATA[self.robot.color]["pano_angle"]
            self.robot.pano_angle = args["pano_angle"]
            
            # args["destination"] = 'p1'
            # args["next_state"] = PanosState(self.robot, self.globals, args)
            args = {
                "destination": self.globals["end_pos"],
                'next_state': EndState(self.robot, self.globals, self.args)
            }
            return NavState(self.robot, self.globals, args)
            #return PanosState(self.robot, self.globals, args)






if __name__ == "__main__":
    robot = Robot()
    robot.initNav()
    fsm = FSM(robot, PreInit, globals)
    fsm.run()

