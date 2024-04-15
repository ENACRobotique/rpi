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
        if time.time() - self.start_time > 5:
            color = "bleu"      # récupérer couleur depuis l'IHM
            if color == "bleu":
                self.robot.resetPos(Pos(200, 225, pi/2))
                args = {"panos": ["p1", "p2", "p3", "p4", "p5", "p6"]}
            else:
                args = {"panos": ["p9", "p8", "p7", "p6", "p5", "p4"]}
            return PanosState(self.robot, self.globals, args)

class PanosState(State):
    def __init__(self, robot: Robot, globals, args={}) -> None:
        super().__init__(robot, globals, args)
    
    def enter(self, prev_state: State | None):
        # s'il reste des panneaux à tourner, y aller
        if len(self.args["panos"]) > 0:
            print(f"Go to {self.args['panos'][0]}")
            self.robot.goToWaypoint(self.args["panos"][0], pi/2)

    def loop(self) -> State | None:
        if len(self.args["panos"]) == 0:
            return EndState(self.robot, self.globals, {})
        if robot.hasReachedTarget():
            # touner panneau
            # partage "args" avec le state "enfant" PanoTurnState
            return PanoTurnState(self.robot, self.globals, self.args)

class PanoTurnState(State):
    def __init__(self, robot: Robot, globals, args={}) -> None:
        super().__init__(robot, globals, args)
    
    def enter(self, prev_state: State | None):
        print(f"tourner panneau {self.args['panos'][0]}...")
        self.prev_state = prev_state
        # TODO se positionner précisemment avec l'aruco
        self.start_time = time.time()

    def loop(self) -> State | None:
        # faire tourner le panneau
        if time.time() - self.start_time > 3:
            return self.prev_state

    def leave(self, next_state: State):
        # le panneau est tourné, on peut l'oublier pour passer au suivant.
        del self.args['panos'][0]

class EndState(State):
    def enter(self, prev_state: State | None):
        print("The End !")


if __name__ == "__main__":
    robot = Robot()
    robot.initNav()
    globals = {"strat_name": "test strat"}
    fsm = FSM(robot, InitState, globals)
    fsm.run()

