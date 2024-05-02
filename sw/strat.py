#!/usr/bin/python3
from fsm import State, FSM
from state_essentials import *
import sys
sys.path.append("../")
from robot import Robot, Pos,Team, Tirette, Strat, THETA_PINCES_BABORD, THETA_PINCES_TRIBORD
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
        Strat.Basique: 'secureJ',
        Strat.Audacieuse: 'midJ'
    },
    Team.BLEU: {
        Strat.Basique: 'secureB',
        Strat.Audacieuse: 'midB'
    }
}

ALT_END_POS = {
    Team.JAUNE: {
        Strat.Basique: 'midJ',
        Strat.Audacieuse: 'basJ'
    },
    Team.BLEU: {
        Strat.Basique: 'midB',
        Strat.Audacieuse: 'basB'
    }
}


#Reminder : Plante params : ['waypoint','azimut']
#Reminder : Depose params : ['waypoint','azimut']
STRAT_DATA = {
    Team.JAUNE: {
        "panos": ["p9", "p8", "p7", "p6", "p5","p4"],
        "pano_angle": 180,
        "plantes":[Plante("planteNE",radians(60))], 
        "pots":[("jardiPotJHaut", DeposeState.Azimut.EAST, THETA_PINCES_BABORD)],
        "depose":[Depose("basJ",radians(-60))]
    },
    Team.BLEU: {
        "panos": ["p1", "p2", "p3", "p4", "p5", "p6"],
        "pano_angle": 0,
        "plantes":[Plante("planteNW",radians(-90-60))],
        "pots":[("jardiPotBHaut", DeposeState.Azimut.WEST, THETA_PINCES_BABORD)],
        "depose":[Depose("basB",radians(-150))]
    }
}


globals = {
    "strat_name": "pano strat",
    "end_pos": None,
    "data": None,
    "match_timeout":88
}

class PreInit(State):
    def loop(self):
        if self.robot.tirette == Tirette.IN:
            return InitState(self.robot, self.globals, {})
        self.robot.buzz(ord('E'))
        time.sleep(0.2)

class InitState(State):
    def enter(self, prev_state: State | None):
        print("Let's get it started in here !")
        print(f"strat name: {self.globals['strat_name']}")
        self.start_time = time.time()
        for i in range(4):
            self.robot.buzz(ord('E'))
            time.sleep(0.1)
            self.robot.buzz(ord('F'))
            time.sleep(0.1)
            self.robot.buzz(ord('G'))
            time.sleep(0.1)
        self.robot.buzz(ord('0'))

    
    def loop(self):
        # tester si tirette tirée
        #self.robot.resetPosFromNav("basB")
        if self.robot.tirette == Tirette.OUT:
            self.globals["match_start_time"] = time.time()
            print(f"start with color {self.robot.color} ans strat {self.robot.strat}!")
            self.globals["end_pos"] = END_POS[self.robot.color][self.robot.strat]
            self.globals["data"] = STRAT_DATA[self.robot.color]
            start_pos = START_POS[self.robot.color][self.robot.strat]
            w, theta = start_pos
            wx, wy = self.robot.nav.getCoords(w)
            rp = Pos(wx, wy, theta)
            self.robot.resetPosFromNav(*start_pos)
            args = {
                "panos": self.globals["data"]["panos"],
                "pano_angle": self.globals["data"]["pano_angle"],
                "plantes": self.globals["data"]["plantes"],
                "pots": self.globals["data"]["pots"],
                "depose": self.globals["data"]["depose"]
            }
            
            self.robot.pano_angle = args["pano_angle"]
            self.robot.updateScore(0)
 
            args_alt = {
                "destination": ALT_END_POS[self.robot.color][self.robot.strat],
                'next_state': EndState(self.robot, self.globals, args)
            }
            args["alternative"] = NavState(self.robot, self.globals, args_alt)
            
            
            if self.robot.strat == Strat.Basique:
                return PanosState(self.robot, self.globals, args)
            
            if self.robot.strat == Strat.Audacieuse:
                
                #farming puis pano
                #args['next_state'] = PanosState(self.robot, self.globals, args)
                return FarmingState(self.robot, self.globals, args)
            
            #return TestState(self.robot, self.globals, args)

class TestState(State):
    def enter(self, prev_state: State | None):
        self.robot.resetPosFromNav('basB',radians(90))
        self.args['destination'] = 'jardiSecureB'
        self.args['orientation'] = radians(-90)
    
    def loop(self):
        self.args['next_state'] = EndState(self.robot, self.globals, self.args)
        return NavState(self.robot, self.globals, self.args)



if __name__ == "__main__":
    robot = Robot()
    robot.initNav()
    fsm = FSM(robot, PreInit, globals)
    fsm.run()

