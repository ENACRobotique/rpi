from fsm import State
from robot import Robot
import time

class NavState(State):
    """ Args : next_state, destination (waypoints only), enemy_alternative_route, timout_enemy"""
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
            self.args['destination'] = self.globals["end_pos"]
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
        if (time.time() - self.robot.aruco_time >= 1) or (abs(self.robot.aruco_y) > 100) or abs(self.robot.aruco_theta) < 40:
            print("fffflllllllaaaaaaaaagggggggggg")
            self.args['flag_bad_aruco'] = True
            return
        else:
            self.args['flag_bad_aruco']  = False
        print(f"aruco cmd used: x = {self.robot.aruco_x}\t y = {self.robot.aruco_y}")
        self.robot.move_rel(self.robot.aruco_x,self.robot.aruco_y) # on se rapproche du pano
    def loop(self) -> State | None:
        # faire tourner le panneau
        if self.args['flag_bad_aruco']:
            return PanosState(self.robot, self.globals, self.args)
        if self.robot.hasReachedTarget():
            if not self.robot.command_sent :
                self.robot.command_sent = True
                self.robot.panoDo(self.robot.commande_pano)
                return PanosState(self.robot, self.globals, self.args)

    def leave(self, next_state: State):
        # le panneau est tourné, on peut l'oublier pour passer au suivant.
        self.robot.command_sent = False
        del self.args['panos'][0]
    
# class AvoidState(State):
#     """ dans le navstate on regarde (indirectement) le topic proximity et on renvoie cet état en conséquence" """
#     def __init__(self, robot: Robot, globals: dict, args: dict) -> None:
#         super().__init__(robot, globals, args)
    
#     def enter(self, prev_state: State | None):
#         self.prev_state = prev_state
#         self.timeout = self.args['timeout']
#         self.init_time = time.time()

#         ...

#     def loop(self) -> State | None:
#         if time.time()-self.init_time >= self.timeout:
#            
#         ...
