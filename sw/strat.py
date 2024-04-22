from fsm import State, FSM
import sys
sys.path.append("../")
from robot import Robot, Pos
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
            color = "bleu"      # récupérer couleur depuis l'IHM
            args = {}
            if color == "bleu":
                self.robot.resetPos(Pos(225, 225, pi/2))
                args["panos"] =  ["p1","p3"] # ["p1", "p2", "p3", "p4", "p5", "p6"]
                args["pano_angle"] = 0
            else:
                args["panos"] = ["p9", "p8", "p7", "p6", "p5", "p4"]
                args["pano_angle"] = 180
            
            self.robot.pano_angle = args["pano_angle"]

            # args["destination"] = 'p1'
            # args["next_state"] = PanosState(self.robot, self.globals, args)
            
            return PanosState(self.robot, self.globals, args)

class PanosState(State):
    def __init__(self, robot: Robot, globals, args={}) -> None:
        super().__init__(robot, globals, args)
    
    def enter(self, prev_state: State | None):
        # s'il reste des panneaux à tourner, y aller
        if len(self.args["panos"]) > 0:
            print(f"Go to {self.args['panos'][0]}")

    def loop(self) -> State | None:
        if len(self.args["panos"]) == 0:
            
            self.args['next_state'] = EndState(self.robot, self.globals, {})
            self.args['destination'] = "basB"
            return NavState(self.robot, self.globals, self.args)

        self.args["destination"] = self.args["panos"][0]
        self.args['next_state'] = PanoTurnState(self.robot, self.globals, self.args)
        return NavState(self.robot, self.globals, self.args)        
        
class PanoTurnState(State):
    def __init__(self, robot: Robot, globals, args={}) -> None:
        super().__init__(robot, globals, args)
    
    def enter(self, prev_state: State | None):
        print(f"tourner panneau {self.args['panos'][0]}...")
        self.prev_state = prev_state
        self.robot.heading(90) # bras // pano
        time.sleep(1)
        print(f"aruco cmd used: x = {self.robot.aruco_x}\t y = {self.robot.aruco_y}")
        self.robot.move_rel(self.robot.aruco_x,self.robot.aruco_y) # on se rapproche du pano
    def loop(self) -> State | None:
        # faire tourner le panneau
        if self.robot.hasReachedTarget():
            if not self.robot.command_sent :
                self.robot.command_sent = True
                self.robot.panoDo(self.robot.commande_pano)
                return PanosState(self.robot, self.globals, self.args)

    def leave(self, next_state: State):
        # le panneau est tourné, on peut l'oublier pour passer au suivant.
        self.robot.command_sent = False
        del self.args['panos'][0]

class NavState(State):
    """ Args : next_state, destination , enemy_alternative_route, timout_enemy"""
    def __init__(self, robot: Robot, globals: dict, args: dict) -> None:
        super().__init__(robot, globals, args)
    
    def enter(self, prev_state: State | None):
        print(f"Navigating to {self.args['destination']}.")
        self.robot.pathFinder(self.args['destination'])
        self.robot.goToWaypoint(self.robot.nav.chemin[0])
    
    def loop(self) -> State | None:

        if self.robot.hasReachedTarget():
            del self.robot.nav.chemin[0]
            if self.robot.isNavDestReached():
                return self.args['next_state']
            self.robot.goToWaypoint(self.robot.nav.chemin[0])


    

class EndState(State):
    def enter(self, prev_state: State | None):
        print("The End !")


if __name__ == "__main__":
    robot = Robot()
    robot.initNav()
    globals = {"strat_name": "test strat"}
    fsm = FSM(robot, InitState, globals)
    fsm.run()

