#!/usr/bin/python3
from fsm import State, FSM
from state_essentials import *
import sys
sys.path.append("../")
from robot import Robot, Pos, Team, Tirette, Strat, THETA_PINCES_BABORD, THETA_PINCES_TRIBORD
import robot
import time
from math import pi

VL53_TIMEOUT = 40

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
        "plantes":[Plante("planteNE",radians(-90-55))], 
        "pots":[("jardiPotJHaut", DeposeState.Azimut.EAST, THETA_PINCES_BABORD)],
        "depose":[Depose("basJ",radians(-45))],
        "jardi": [Jardi("jardiSecureJ",DeposeState.Azimut.NORTH)]
    },
    Team.BLEU: {
        "panos": ["p1", "p2", "p3", "p4", "p5", "p6"],
        "pano_angle": 0,
        "plantes":[Plante("planteNW",radians(-35))],
        "pots":[("jardiPotBHaut", DeposeState.Azimut.WEST, THETA_PINCES_BABORD)],
        "depose":[Depose("basB",radians(-90-45))],
        "jardi":[Jardi("jardiSecureB",DeposeState.Azimut.NORTH)]
    }
}


globals = {
    "strat_name": "pano strat",
    "end_pos": None,
    "alt_end":None,
    "data": None,
}

class PreInit(State):
    def enter(self, prev_state):
        self.start_time = time.time()
        self.robot.lcd.set_page(self.robot.status_page)

    def loop(self):
        while True:
            if all(self.robot.vl53_started.values()):
                note = ord('E')
                period = 0.5
                vl53_status = "VL OK"
            elif time.time() - self.start_time > VL53_TIMEOUT:
                note = ord('B')
                period = 1
                vl53_status = "VL TO"
            else:
                note = ord('C')
                period = 1.5
                vl53_status = "VL WA"
            
            if self.robot.nb_pos_received < 10:
                note += 7
                period *= 2
                base_status = "BASE WA"
            else:
                base_status = "BASE OK"
            
            self.robot.status_page.set_text(f"{vl53_status} | {base_status}")
            self.robot.buzz(note)
            time.sleep(period)
            if self.robot.tirette == Tirette.IN:
                yield InitState(self.robot, self.globals, {})
        yield None

class InitState(State):
    def enter(self, prev_state: State | None):
        self.robot.logger.info("Let's get it started in here !")
        self.robot.logger.info(f"strat name: {self.globals['strat_name']}")
        self.start_time = time.time()
        for i in range(4):
            self.robot.buzz(ord('F'))
            time.sleep(0.1)
        
    
    def loop(self):
        while True:
            # tester si tirette tirée
            
            if self.robot.tirette == Tirette.OUT:
                self.robot.tempsDebutMatch = time.time()
                self.robot.logger.info(f"start with color {self.robot.color} ans strat {self.robot.strat}!")
                self.globals["end_pos"] = END_POS[self.robot.color][self.robot.strat]
                self.globals["data"] = STRAT_DATA[self.robot.color]
                self.globals["alt_end"] = ALT_END_POS[self.robot.color][self.robot.strat]
                start_pos = START_POS[self.robot.color][self.robot.strat]
                w, theta = start_pos
                wx, wy = self.robot.nav.getCoords(w)
                rp = Pos(wx, wy, theta)
                self.robot.resetPosFromNav(*start_pos)

                #self.robot.logger.info("position robot", self.robot.pos.x, self.robot.pos.y, self.robot.pos.theta)
                args = {
                    "panos": self.globals["data"]["panos"],
                    "pano_angle": self.globals["data"]["pano_angle"],
                    "plantes": self.globals["data"]["plantes"],
                    "pots": self.globals["data"]["pots"],
                    "depose": self.globals["data"]["depose"],
                    "jardi": self.globals["data"]["jardi"]
                }
                
                self.robot.pano_angle = args["pano_angle"]
                self.robot.updateScore(0)
    
                args_alt = {
                    "destination": self.globals["alt_end"],
                    'next_state': EndState(self.robot, self.globals, args)
                }
                args["alternative"] = NavState(self.robot, self.globals, args_alt)
                
                
                if self.robot.strat == Strat.Basique:
                    #yield TestState(self.robot, self.globals, args)
                    #self.lidar_time = time.time()
                    #while time.time() - self.lidar_time <= LIDAR_TIME:
                    #   yield None
                    #self.robot.recallageLidar(4000,True)
                    yield PanosState(self.robot, self.globals, args)
                
                if self.robot.strat == Strat.Audacieuse:
                    
                    #farming puis pano
                    #args['next_state'] = PanosState(self.robot, self.globals, args)
                    yield FarmingState(self.robot, self.globals, args)
            yield None
            

class TestState(State):
    def enter(self, prev_state: State | None):
        self.robot.resetPosFromNav('secureB',-pi/2)
        self.args['destination'] = self.args['plantes'][0].waypoint
        self.args['orientation'] = self.args['plantes'][0].azimut
    
    def loop(self):
        self.robot.setActionneur(Actionneur.Pince3, ValeurActionneur.OpenPince3)
        time.sleep(1)
        self.robot.setActionneur(Actionneur.Pince3, ValeurActionneur.ClosePince3)
        time.sleep(1)
        yield None
        
        #self.robot.logger.info("test fini!")
        #return NavState(self.robot, self.globals, self.args)
        #self.args['next_state'] = EndState(self.robot, self.globals, self.args)
        #return NavState(self.robot, self.globals, self.args)



if __name__ == "__main__":
    robot = Robot("strat")
    robot.initNav()
    fsm = FSM(robot, PreInit, EndState, globals)
    fsm.run()

