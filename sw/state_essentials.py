from fsm import State
from robot import Robot,Actionneur,ValeurActionneur
from common import Pos
import time
from enum import Enum
from math import pi

def timeout(init_time,timeout):
    """Return True if timeout """
    return time.time() - init_time  >= timeout


class NavState(State):
    """ Args : next_state, destination (waypoints only), enemy_alternative_route, timeout"""

    class MoveStatus(Enum):
        MOVING = 1
        STOPPED = 2

    def __init__(self, robot: Robot, globals: dict, args: dict) -> None:
        super().__init__(robot, globals, args)
    
    def enter(self, prev_state: State | None):
        print(f"Navigating to {self.args['destination']}.")
        self.robot.pathFinder(self.args['destination'])
        self.move_status = self.MoveStatus.STOPPED
        self.t_stop = time.time()
        if "timeout" not in self.args:
            self.args["timeout"] = 5 # default timeout 
    
    def loop(self) -> State | None:
        
        if timeout(self.globals["match_start_time"],self.globals["match_timeout"]):
            self.robot.updateScore(-10)
            return EndState(self.robot, self.globals, self.args)
        x,y = self.robot.nav.getCoords(self.robot.nav.chemin[0])
        
        if self.robot.obstacle_in_way(Pos(x=x,y=y,theta=0)) :
            if self.move_status == self.MoveStatus.STOPPED:# wait timeout before doing other planned action
                #print(f"obstacle int the way to '{self.robot.nav.chemin[0]}'")
                if time.time() - self.t_stop > self.args["timeout"] and "alternative" in self.args:
                    return self.args["alternative"]
                
            elif self.move_status == self.MoveStatus.MOVING: # Stop the robot and start timeout timer
                self.t_stop = time.time()
                self.robot.setTargetPos(self.robot.pos)
                self.move_status = self.MoveStatus.STOPPED
        else:
            if self.move_status == self.MoveStatus.STOPPED:
                self.robot.goToWaypoint(self.robot.nav.chemin[0])
                self.move_status = self.MoveStatus.MOVING

            elif self.move_status == self.MoveStatus.MOVING: 
                if self.robot.hasReachedTarget():
                    del self.robot.nav.chemin[0]
                    if self.robot.isNavDestReached():
                        return self.args['next_state']
                    self.robot.goToWaypoint(self.robot.nav.chemin[0])

    
class EndState(State):
    def enter(self, prev_state: State | None):
        self.robot.updateScore(10)
        #si le robot entre dans le End state suite a un timeout de fin de match ( avant les PAMI )
        # cela signifie donc qu'on est pas dans la zone finale qui rapporte les 10 point
        # alors il faudra faire score -=10 avant de rentrer dans l'etat ( donc un timeout force le score -10 et le endstate)
        # display score on LCD 
        print("The End !")


class PanosState(State):
    def __init__(self, robot: Robot, globals, args={}) -> None:
        super().__init__(robot, globals, args)
    
    def enter(self, prev_state: State | None):
        # s'il reste des panneaux à tourner, y aller
        if len(self.args["panos"]) > 0:
            print(f"Go to {self.args['panos'][0]}")

    def loop(self) -> State | None:
        
        if timeout(self.globals["match_start_time"],self.globals["match_timeout"]):
            self.robot.updateScore(-10)
            return EndState(self.robot, self.globals, self.args)
        
        if len(self.args["panos"]) == 0:
            
            return FarmingState(self.robot, self.globals, self.args)

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
        #print(f"aruco cmd used: x = {self.robot.aruco_x}\t y = {self.robot.aruco_y}")
        self.robot.move_rel(self.robot.aruco_x,self.robot.aruco_y) # on se rapproche du pano
    
    def loop(self) -> State | None:
        
        if timeout(self.globals["match_start_time"],self.globals["match_timeout"]):
            self.robot.updateScore(-10)
            return EndState(self.robot, self.globals, self.args)
        
        # faire tourner le panneau
        if self.args['flag_bad_aruco']:
            return PanosState(self.robot, self.globals, self.args)
        if self.robot.hasReachedTarget():
            if not self.robot.command_sent :
                self.robot.command_sent = True
                self.robot.panoDo(self.robot.commande_pano)
                self.robot.updateScore(5)
                return PanosState(self.robot, self.globals, self.args)

    def leave(self, next_state: State):
        # le panneau est tourné, on peut l'oublier pour passer au suivant.
        self.robot.command_sent = False
        del self.args['panos'][0]
    
class FarmingState(State):
    def __init__(self, robot: Robot, globals, args={}) -> None:
        super().__init__(robot, globals, args)
    def enter(self, prev_state: State | None):
        #if len(self.args["plantes"]) > 0:
        print(f"Go to {self.args['plantes'][0][0]}")

        time.sleep(1)
        self.robot.recallageLidar()

    def loop(self) -> State | None:
        
        if timeout(self.globals["match_start_time"],self.globals["match_timeout"]):
            self.robot.updateScore(-10)
            return EndState(self.robot, self.globals, self.args)
        
        if len(self.args["plantes"]) == 0:
            
            self.args['next_state'] = EndState(self.robot, self.globals, {})
            self.args['destination'] = self.globals["end_pos"]
            return NavState(self.robot, self.globals, self.args)

        self.args["destination"] = self.args["plantes"][0][0]
        self.args['next_state'] = PlantesState(self.robot, self.globals, self.args)
        return NavState(self.robot, self.globals, self.args)

class PlantesState(State):
    """TEMPORAIRE """
    class MoveStatus(Enum):
        MOVING = 1
        STOPPED = 2

    def __init__(self, robot: Robot, globals, args={}) -> None:
        super().__init__(robot, globals, args)
    
    def enter(self, prev_state: State | None):
        print(f"Chercher plantes {self.args['plantes'][0][0]}...")
        self.prev_state = prev_state
        self.robot.heading(self.args['plantes'][0][3])
        self.move_status = self.MoveStatus.STOPPED
        time.sleep(1)
    
    def loop(self) -> State | None:
        
        if timeout(self.globals["match_start_time"],self.globals["match_timeout"]):
            self.robot.updateScore(-10)
            return EndState(self.robot, self.globals, self.args)
        
        # s'alligner en utilisant les VL53
        if self.robot.hasReachedTarget():
            if not self.robot._face_plante:
                print("je m'alligne ")
                self.robot.setActionneur(Actionneur.Pince1,ValeurActionneur.OpenPince1)
                time.sleep(0.1)
                self.robot.setActionneur(Actionneur.Pince2,ValeurActionneur.OpenPince2)
                time.sleep(0.1)
                self.robot.setActionneur(Actionneur.AxL,ValeurActionneur.DownAxL)
                time.sleep(1)
                if self.move_status == self.MoveStatus.STOPPED :
                    self.robot.move(200,(self.args['plantes'][0][1]))
                    self.move_status = self.MoveStatus.MOVING
                    time.sleep(2)

        if self.move_status == self.MoveStatus.MOVING:
                if self.robot.hasReachedTarget():
                    print("je suis aligné")
                    self.robot._face_plante = True
                    self.move_status = self.MoveStatus.STOPPED
        
        #prendre la plante
        if self.robot._face_plante:
            if not self.robot._plante:
                
                if self.robot.hasReachedTarget():
                    self.move_status = self.MoveStatus.STOPPED
                    print("je prend")
                    self.robot.setActionneur(Actionneur.Pince1,ValeurActionneur.ClosePince1)
                    time.sleep(0.1)
                    self.robot.setActionneur(Actionneur.Pince2,ValeurActionneur.ClosePince2)
                    time.sleep(1)
                    self.robot.setActionneur(Actionneur.AxL,ValeurActionneur.UpAxL)
                    self.robot._plante = True
                    print("j'ai pris la plante")
        
        if self.robot._plante:
                print( "je vais déposer ")
                self.robot.recallageLidar()
                
                self.args["destination"] = self.args["depose"][0][0]
                self.args['next_state'] = DeposeState(self.robot, self.globals, self.args)
                return NavState(self.robot, self.globals, self.args)

    def leave(self, next_state: State):
        # les plantes sont rammasée, on peut l'oublier pour passer au suivant.
        del self.args['plantes'][0]



class DeposeState(State):
    """TEMPORAIRE """
    def __init__(self, robot: Robot, globals, args={}) -> None:
        super().__init__(robot, globals, args)
    
    def enter(self, prev_state: State | None):
        print(f" Déposer Butin {self.args['depose'][0][0]}...")
        self.prev_state = prev_state
        self.robot.heading(self.args['depose'][0][1])
    
    def loop(self) -> State | None:
        
        if timeout(self.globals["match_start_time"],self.globals["match_timeout"]):
            self.robot.updateScore(-10)
            return EndState(self.robot, self.globals, self.args)
        
        # prendre la plante
        if self.robot.hasReachedTarget():
            self.robot.setActionneur(Actionneur.Pince1,ValeurActionneur.OpenPince1)
            time.sleep(0.1)
            self.robot.setActionneur(Actionneur.Pince2,ValeurActionneur.OpenPince2)

            if self.robot.hasReachedTarget():
                self.robot.setActionneur(Actionneur.Pince1,ValeurActionneur.ClosePince1)
                time.sleep(0.1)
                self.robot.setActionneur(Actionneur.Pince2,ValeurActionneur.ClosePince2)
                time.sleep(0.1)
                self.robot.setActionneur(Actionneur.AxL,ValeurActionneur.UpAxL)

                self.args["destination"] = self.globals["end_pos"]
                self.args['next_state'] = EndState(self.robot, self.globals, self.args)
                return NavState(self.robot, self.globals, self.args)

# class PotState(State):
#     """TEMPORAIRE """
#     def __init__(self, robot: Robot, globals, args={}) -> None:
#         super().__init__(robot, globals, args)
    
#     def enter(self, prev_state: State | None):
#         print(f"Chercher pot {self.args['pots'][0][0]}...")
#         self.prev_state = prev_state
#         self.robot.heading(self.args['pots'][0][1])
    
#     def loop(self) -> State | None:
        
#         if timeout(self.globals["match_start_time"],self.globals["match_timeout"]):
#             self.robot.updateScore(-10)
#             return EndState(self.robot, self.globals, self.args)
        
#         # poser la plante dans le pot
#         if self.robot.hasReachedTarget():
#             self.robot.setActionneur(Actionneur.AxL,ValeurActionneur.UpAxL)
#             self.robot.move(50,self.args['pots'][0][1])
#             self.robot.setActionneur(Actionneur.Pince2,ValeurActionneur.OpenPince2)
#             if self.robot.hasReachedTarget():
#                 self.robot.setActionneur(Actionneur.Pince2,ValeurActionneur.ClosePince2)
#                 time.sleep(0.1)
#                 self.robot.setActionneur(Actionneur.AxL,ValeurActionneur.UpAxL)
#                 return PotState(self.robot, self.globals, self.args)

#     def leave(self, next_state: State):
#         # les pots sont rammassé, on peut l'oublier pour passer au suivant.
#         del self.args['pots'][0]
    
