import py_trees
import sys
sys.path.append("../..")
from sw.IO.actionneurs import *
from robot import *
from bt_essentials import get_bb_robot

##### Behavior tree pour l'automatisation #####

class LiftPlanche(py_trees.behaviour.Behaviour):
    def __init__(self, pos:int):
        pos_info = "DOWN"
        if pos == UP:
            pos_info = "UP"
        if pos == MID:
            pos_info = "MID"
        super().__init__(name=f"Lift {pos_info} planche")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.pos = pos
        self.done = False
        self.pos_info = pos_info

    def initialise(self) -> None:
        if not self.done:
            print(f"Lift {self.pos_info} planche")
            self.robot.actionneurs.liftPlanches(self.pos)
        
    def update(self):
        if self.robot.actionneurs.Servo_IO.isMoving(Actionneur.PlancheGauche.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.PlancheDroit.value):
            return py_trees.common.Status.RUNNING
        else:
            self.done = True
            return py_trees.common.Status.SUCCESS



class LiftConserve(py_trees.behaviour.Behaviour):
    def __init__(self, pos:ValeurActionneur):
        super().__init__(name=f"Lift {pos.name} Conserve")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.pos = pos

    def initialise(self) -> None:
        print(f"Lift {self.pos} Conserve")
        self.robot.actionneurs.liftConserve(self.pos)

    def update(self):
        if self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AscenseurAimant.value):
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS
    
class MoveRentreur(py_trees.behaviour.Behaviour):
    def __init__(self, position:bool):
        pos = "OUTSIDE"
        if position:
            pos = "INSIDE"
        super().__init__(name=f"Rentreur going {pos}")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.position = position
        self.done = False
        self.pos = pos

    def initialise(self) -> None:
        print(f"Rentreur going {self.pos}")
        self.robot.actionneurs.moveRentreur(self.position)

    def update(self):
        if self.robot.actionneurs.Servo_IO.isMoving(Actionneur.Rentreur.value):
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

class GrabHighConserve(py_trees.behaviour.Behaviour):
    def __init__(self, grab : bool):
        stat = "Dropping"
        if grab:
            stat = "Grabbing"
        super().__init__(name=f"{stat} High Conserve")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.grab = grab
        self.done = False
        self.stat= stat

    def initialise(self) -> None:
        print(f"{self.stat} High Conserve")
        self.robot.actionneurs.grabHighConserve(self.grab)

    def update(self):
        if self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AimantHautGauche.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AimantHautDroit.value):
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS



class GrabLowConserve(py_trees.behaviour.Behaviour):
    def __init__(self, grab : bool):
        stat = "Dropping"
        if grab:
            stat = "Grabbing"
        super().__init__(name=f"{stat} Low Conserve")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.grab = grab
        self.done = False
        self.stat = stat
    
    def initialise(self) -> None:
        print(f"{self.stat} Low Conserve")
        self.robot.actionneurs.grabLowConserve(self.grab) 

    def update(self):
        if self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AimantBasGauche.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AimantBasDroit.value):
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS
    
class LockPlanche(py_trees.behaviour.Behaviour):
    def __init__(self, lock : bool):
        stat = "Unlocking"
        if lock:
            stat = "Locking"
        super().__init__(name=f"{stat} Upper Planche")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.lock = lock
        self.done = False
        self.stat = stat

    def initialise(self) -> None:
        print(f"{self.stat} Upper Planche")
        self.robot.actionneurs.lockPlanche(self.lock)

    def update(self):
        if self.robot.actionneurs.Servo_IO.isMoving(Actionneur.VerrouPince.value):
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS
    
class DeployMacon(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name=f"Deploying Macon")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.deployed = False

    def initialise(self) -> None:
        if not self.deployed :
            print("Deploying Macon")
            self.robot.actionneurs.deployMacon()

    def update(self):
        if self.robot.actionneurs.Servo_IO.isMoving(Actionneur.BrasPince.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.VerrouPince.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AimantHautGauche.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AimantHautDroit.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AimantBasGauche.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AimantBasDroit.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.PlancheGauche.value) or self.robot.actionneurs.Servo_IO.isMoving(Actionneur.PlancheDroit.value):
            return py_trees.common.Status.RUNNING
        else:
            self.deployed = True
            return py_trees.common.Status.SUCCESS

class waitCalibration(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name=f"Waiting Calibration")
        self.bb, self.robot, self.world = get_bb_robot(self)
    
    def initialise(self) -> None:
        self.robot.actionneurs.calibrateLift()

    def update(self):
        if self.robot.actionneurs.liftCalibrated:
            print("Calibrated !")
            return py_trees.common.Status.SUCCESS
        print("Waiting Calibration")
        return py_trees.common.Status.FAILURE


class AlignPlanches(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="Align Planches")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.done = False
    
    # def initialise(self):
    #     print("J'essaie de m'aligner !!!!!!!!!!!!!!!!!!!!!!!")
    # def terminate(self, new_status: py_trees.common.Status) -> None:
    #     print("J'ai fini de m'aligner !!!!!!!!!!!!!!!!!!!!!!!")

    def update(self):
        if not self.done:
            try:
                a, d, _ = self.robot.vl53_planches2()
            except TypeError:
                return py_trees.common.Status.RUNNING
            if a < radians(-3):
                # rotate left
                self.robot.locomotion.set_speed(Speed(0, 0, -0.1))
            elif a > radians(3):
                # rotate right
                self.robot.locomotion.set_speed(Speed(0, 0, 0.1))
            else:
                print("Planches aligned")
                self.robot.locomotion.set_speed(Speed(0, 0, 0))
                self.done = True
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
        


class AlignConserves(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="Align Conserves")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.done = False
        self.confirmation_align = 0
    def initialise(self) -> None:
        if not self.done:
            self.confirmation_align = 0
            self.initial_pos = self.robot.pos
            print('Alignement Conserve')
            self.last_time = time.time()

    def update(self):# GROS WIP ÇA MARCHE PAS ENCORE OSCOUR
        necessary_waiting_time = 1.5 #seconds
        x,y = None, None
        if not self.done:
            if self.robot.hasReachedTarget():
                if abs(time.time() - self.last_time) > necessary_waiting_time:
                    print("Should see smth")
                    x,y = self.robot.cameras.cam_cons()
                    if y is not None:
                        if abs(y) < 8: # mm, à régler
                            self.confirmation_align+=1
                            print("Je suis aligné !!!")
                            self.robot.buzz(ord('G'))
                            self.robot.move(0, 0, 0)
                            # if self.confirmation_align>5:
                            self.done = True
                            return py_trees.common.Status.SUCCESS
                        else:
                            dir = -y/abs(y)
                            # print(f"Bouge de {y} direction {'droite' if dir<0 else 'gauche'}\n")
                            self.robot.move(abs(y), dir*radians(90), 150)
                            # self.confirmation_align = 0
            else:
                self.last_time = time.time()
            # else:
                # if self.robot.hasReachedTarget():
                #     self.robot.setTargetPos(self.robot.pos)
                #     print("Je reviens à ma place")
                #On voit rien du tout
                # self.robot.move(0, 0, 0)
                # return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING
    
        # ix, d = self.robot.detect_best_conserve(Actionneur.AimantBasGauche)
        # ix2, d = self.robot.detect_best_conserve(Actionneur.AimantBasDroit)
        # if (ix < 3 and ix2 > 4) or (ix > 4 and ix2 < 3):
        #     return py_trees.common.Status.FAILURE

        # if ix < 3 or ix2 < 3:
        #     # move left
        #     self.robot.locomotion.set_speed(Speed(0, 15, 0))
        # elif ix > 4 or ix2 > 4:
        #     # move right
        #     self.robot.locomotion.set_speed(Speed(0, -15, 0))
        # else:
        #     print("Conserves aligned")
        #     return py_trees.common.Status.SUCCESS
        # return py_trees.common.Status.RUNNING


class Aligne_conserve_seul(py_trees.behaviour.Behaviour):
    def __init__(self, actionneur:Actionneur):
        super().__init__(name="Align Conserve Seul")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.actionneur = actionneur
        if self.actionneur == Actionneur.AimantBasGauche:
            self.sign = 1
        else:
            self.sign = -1
    def update(self):
        ix, d = self.robot.detect_one_conserve(self.actionneur)
        if d > 200:
            self.robot.locomotion.set_speed(Speed(0, 0, self.sign *0.5))  
        else:
            if ix < 3:
                # move left
                self.robot.locomotion.set_speed(Speed(0, 0, 0.2))
            elif ix > 4:
                # move right
                self.robot.locomotion.set_speed(Speed(0, 0, -0.2))
            else:
                print("Conserve aligned")
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING



class AvanceConserve(py_trees.behaviour.Behaviour):
    def __init__(self, actionneur:Actionneur):
        super().__init__(name="Avance Conserve")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.actionneur = actionneur
    
    def initialise(self):
        _, d = self.robot.detect_one_conserve(self.actionneur)
        self.robot.move(d-40, 0, 200)

    def update(self):
        if self.robot.hasReachedTarget():
            print("Avance Conserve done")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
    

class AvancePlanches(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="Avance Planches")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.done = False
        self.x = None
    
    def initialise(self):
        # try:
        #     _, d, _ = self.robot.vl53_planches2()
        #     self.robot.move(d-40, 0, 100)
        # except TypeError:
        #     return py_trees.common.Status.RUNNING
        if not self.done:
            self.last_time = time.time()
            print("Je m'avance vers les planches")
    
    def update(self):
        necessary_waiting_time = 1
        x = None
        if self.robot.hasReachedTarget():
            if abs(time.time() - self.last_time) > necessary_waiting_time:
                print("Should see smth")
                x,y = self.robot.cameras.cam_cons()
                if x is not None:
                    if abs(x)< 10:
                        print("Je suis assez près des planches")
                        self.done = True
                        return py_trees.common.Status.SUCCESS
                    else:
                        self.robot.move(x, 0, 100)
        else:
            self.last_time=time.time()
        return py_trees.common.Status.RUNNING

        # if self.robot.hasReachedTarget():
        #     print("Avance Planches done")
        #     return py_trees.common.Status.SUCCESS
        # return py_trees.common.Status.RUNNING

class LiftBanderole(py_trees.behaviour.Behaviour):
    def __init__(self,up:bool):
        super().__init__(name="Lift Banderole")
        self.bb, self.robot, self.world = get_bb_robot(self)
        self.up = up
        self.done = False
    
    def initialise(self):
        if self.done:
            return
        self.robot.actionneurs.liftBanderole(self.up)

    def update(self):
        if self.done:
            return py_trees.common.Status.SUCCESS
        if self.robot.actionneurs.Servo_IO.isMoving(Actionneur.AscenseurBanderolle.value):
            return py_trees.common.Status.RUNNING
        else:
            if self.up == DOWN:
                self.robot.updateScore(20)
            self.done = True
            return py_trees.common.Status.SUCCESS


