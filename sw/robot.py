#!/usr/bin/env python3
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
import time
from math import sqrt, pi, cos, sin, atan2, radians,degrees
import sys
import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb
import generated.messages_pb2 as base_pb
import common
from common import Pos, Speed, dist_to_line # tkt ça marche

from enum import Enum
from dataclasses import dataclass
import numpy as np
import nav 

import lcd_client as lcd

XY_ACCURACY = 8  # mm
THETA_ACCURACY = radians(4) # radians
AVOIDANCE_OBSTACLE_MARGIN = 500 #in mm.  Standard robot enemy radius is 22 cm

THETA_PINCES_BABORD = radians(60)  #pinces babord
THETA_PINCES_TRIBORD = radians(-60)

# avoidance bounds 
BOUNDS = (-100,400,-250,250)

class Team(Enum):
    BLEU = 1
    JAUNE = 2

class Tirette(Enum):
    IN = 0
    OUT = 1

class Frame(Enum):
    TABLE = 0
    ROBOTCENTRIC = 1
    ROBOT = 2

class Strat(Enum):
    Basique = 0
    Demo = 1
    Homologation = 2
    Audacieuse = 3
    ShowOff = 4



class Actionneur(Enum):
    Pince1 = 3
    Pince2 = 2
    Pince3 = 5
    Pince4 = 6
    Bras   = 4
    Pano   = 1
    AxBabord    = 7
    AxTribord    = 8

class ValeurActionneur(Enum):
    InitPano = 1500

    OpenPince1 = 1130
    OpenPince2 = 1800
    OpenPince3 = 1600
    OpenPince4 = 1600
    
    ClosePince1 = 750
    ClosePince2 = 1400
    ClosePince3 = 1600
    ClosePince4 = 1600
    
    DownBras = 1960
    UpBras = 950
    
    UpAxBabord = 800
    UpAxTribord = 200

    MidAxBabord = 500
    MidAxTribord = 640   

    DownAxBabord = 40
    DownAxTribord = 1010

PANO_CONVERT = 90/105 # 2.5 cm
PANO_OFFSET = 125 # mm

class Robot:
    """Classe dont le but est de se subscribe à ecal pour avoir une représentation de l'état du robot
    
    Créez un objet de cette classe si vous avez besoin de connaître l'état du robot."""
    def __init__(self):
        """x et y sont en mètres
        theta est en radian"""
        ecal_core.initialize(sys.argv, "robotStateHolder")
        self.pos = Pos(0, 0, 0)
        self.speed = Speed(0, 0, 0)
        self.last_target = Pos(0, 0, 0)
        self.nav = nav.Nav()
        self.pano_angle = 0
        self.aruco_time = 0
        self.command_sent = False
        self.lidar_pos = Pos(0,0,0)

        self.aruco_y = 0
        self.aruco_x = 0
        self.aruco_theta = 0

        self.color = Team.BLEU
        self.tirette = Tirette.OUT
        self.strat = Strat.Basique
        self.score = 0
        self.obstacles = []

        self._pid_gains = [0, 0, 0]     # Just for manual setting of PIDS

        #self.tirette = robot_pb.IHM.T_NONE
        #self.color = robot_pb.IHM.C_NONE
        #self.proximityStatus = None

        #self.lastCommandNumber = None
        #self.lastFinishedActionNumber = None
        #self.pointsEstimes =0

        self.tempsDebutMatch = None

        m=lcd.Menu("Robot", None)
        strat_choices_page = lcd.Choice("Strat", m, [s for s in Strat], self.set_strat)
        detect_range_page = lcd.Number("Dist detection", m, 20, 150, None)
        self.pos_page = lcd.Text("Position", m, "---")
        self.score_page = lcd.Text("Score", m, "0")
        pid_page = lcd.Menu("PID", m)
        m.add_subpages(strat_choices_page, detect_range_page, self.pos_page, self.score_page, pid_page)

        kp_page = lcd.Number("Kp", pid_page, 0, 10, lambda x: self.set_pid_gain(0, x))
        ki_page = lcd.Number("Ki", pid_page, 0, 1, lambda x: self.set_pid_gain(1, x))
        kd_page = lcd.Number("Kd", pid_page, 0, 5, lambda x: self.set_pid_gain(2, x))
        pid_page.add_subpages(kp_page, ki_page, kd_page)
        self.lcd = lcd.LCDClient(m, self.on_lcd_event, self.on_lcd_state)
        self.lcd.start()

        ### SUB ECAL ###

        #self.matchReportSub = ProtoSubscriber('match_start',robot_pb.Match)
        #self.matchReportSub.set_callback(self.onReceiveMatchStarted)

        self.positionReportSub = ProtoSubscriber("odom_pos",robot_pb.Position)
        self.positionReportSub.set_callback(self.onReceivePosition)

        self.speedReportSub = ProtoSubscriber("odom_speed",robot_pb.Speed)
        self.speedReportSub.set_callback(self.onReceiveSpeed)

        self.pano_sub = ProtoSubscriber("aruco",robot_pb.Position_aruco)
        self.pano_sub.set_callback(self.aruco)
        
        self.amalgame_sub = ProtoSubscriber("amalgames", lidar_pb.Amalgames)
        self.amalgame_sub.set_callback(self.detection)

        self.lidar_sub = ProtoSubscriber("smooth_pos", robot_pb.Position)
        self.lidar_sub.set_callback(self.onLidar)

        #self.setPositionSub = ProtoSubscriber("set_position", robot_pb.Position)
        #self.setPositionSub.set_callback(self.onSetTargetPostition)

        #self.proximitySub = ProtoSubscriber("proximity_status",lidar_pb.Proximity)
        #self.proximitySub.set_callback(self.onProximityStatus)
        
        ### PUB ECAL ###
        self.set_target_pos_pub = ProtoPublisher("set_position", robot_pb.Position)
        self.reset_pos_pub = ProtoPublisher("reset", robot_pb.Position)

        self.IO_pub = ProtoPublisher("Actionneur",robot_pb.IO)

        self.color_pub = ProtoPublisher("color", robot_pb.Side)

        self.pid_pub = ProtoPublisher("pid_gains", base_pb.MotorPid)

        #self.claw_pub = ProtoPublisher("set_pince", robot_pb.SetState)
        #self.score_pub = ProtoPublisher("set_score", robot_pb.Match)
                
        #self.slow_pub = ProtoPublisher("slow",robot_pb.no_args_func_)
        #self.stop_pub = ProtoPublisher("stop",robot_pb.no_args_func_)
        #self.resume_pub = ProtoPublisher("resume",robot_pb.no_args_func_)

        self.debug_pub =StringPublisher("debug_msg")
        self.objects_pubs = [ProtoPublisher(f"Obstacle{i}",robot_pb.Position) for i in range(3)]
        time.sleep(1)

        self.nav.initialisation()
        self.initActionneur()

    def set_color(self, c):
        self.color = c
        msg = robot_pb.Side()
        if self.color == Team.JAUNE:
            msg.color = robot_pb.Side.Color.YELLOW
            self.lcd.red = True
            self.lcd.green = True
            self.lcd.blue = False
        else:
            msg.color = robot_pb.Side.Color.BLUE
            self.lcd.red = False
            self.lcd.green = False
            self.lcd.blue = True
        print("Equipe : ",c)
        self.color_pub.send(msg)

    def __repr__(self) -> str:
        return "Cooking Mama's status storage structure"


    @staticmethod
    def normalize(angle):
        while angle >= pi:
            angle-=2*pi
        while angle < -pi:
            angle += 2*pi
        return angle
    

    def on_lcd_event(self, event):
        if event.button == robot_pb.LCDEvent.Button.COLOR and event.value == 1:
            color = Team.BLEU if self.color == Team.JAUNE else Team.JAUNE
            self.set_color(color)
        elif event.button == robot_pb.LCDEvent.Button.TIRETTE:
            self.tirette = Tirette(event.value)

    
    def on_lcd_state(self, state):
        self.tirette = Tirette(state.tirette)


    def hasReachedTarget(self):
        d=sqrt((self.pos.x-self.last_target.x)**2 + (self.pos.y-self.last_target.y)**2)
        return (d <= XY_ACCURACY) and (abs(self.pos.theta - self.last_target.theta) <= THETA_ACCURACY)
    

    def setTargetPos(self, pos: Pos, frame=Frame.TABLE,blocking=False, timeout = 10):
        """Faire setTargetPos(Pos(x,y,theta)) en mm et angle en radian """

        if frame == Frame.ROBOTCENTRIC:
            pos += self.pos
        elif frame == Frame.ROBOT:
            pos = pos.from_frame(self.pos)
        pos.theta = Robot.normalize(pos.theta)
        pb_pos = pos.to_proto()
        #print(f"go to: {pos}")
        self.set_target_pos_pub.send(pb_pos)
        self.last_target = pos
        
        if blocking :
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.hasReachedTarget():
                    return True
                time.sleep(0.1)
            return False
        
            
            




    def move(self, distance, direction, blocking=False, timeout = 10):
        
        frame_pince = Pos(0, 0, direction)
        target = Pos(distance, 0, -direction).from_frame(frame_pince)
        self.setTargetPos(target, Frame.ROBOT,blocking, timeout)
    
    def move_rel(self,x,y,blocking=False, timeout = 10):
        if x : 
            self.move(sqrt(x**2+y**2),atan2(y,x),blocking, timeout)
        else :
            self.move(y,pi/2*np.sign(y),blocking, timeout)
    
    def heading(self,angle,blocking=False, timeout = 10):
        """ S'oriente vers la direction donnée
         \nArgs float:theta en radian""" 
        self.setTargetPos(Pos(self.pos.x,self.pos.y,angle),Frame.TABLE, blocking, timeout)
    
    def rotate(self,angle,blocking=False, timeout = 10):
        """ Rotation en relatif
         \nArgs, float:theta en radians """
        self.heading(self.pos.theta + angle,blocking=False, timeout = 10)
    
    def resetPos(self, position: Pos, timeout=2):
        print(f"Pos to reset to : {position.x},\t{position.y}, \t{position.theta} ")
        self.reset_pos_pub.send(position.to_proto())
        start_time = time.time()
        #self.pos = position
        while time.time() - start_time < timeout:
            
            if self.pos.distance(position) < 1:
                
                time.sleep(0.1)
                break
    
    def updateScore(self,points):
        self.buzz(ord('D'))
        time.sleep(0.1)
        self.buzz(ord('G'))
        time.sleep(0.1)
        self.buzz(ord('0'))
        self.score += points
        self.score_page.set_text(f"Score",f"{self.score}")
        self.lcd.set_page(self.score_page)
    
    def buzz(self,tone):
        """Args , string:tone
        \ntone : ['A'-'G'] + 7*octave : note à cette octave (0<=octave<=2)"""
        self.lcd.buzz = tone
        self.lcd.display()

    def onSetTargetPostition (self, topic_name, msg, timestamp):
        """Callback d'un subscriber ecal. Actualise le dernier ordre de position"""
        self.last_target = Pos.from_proto(msg)

    def onReceivePosition (self, topic_name, msg, timestamp):
        """Callback d'un subscriber ecal. Actualise la position du robot"""
        self.pos = Pos.from_proto(msg)
        self.pos_page.set_text(f"x:{msg.x:.0f} y:{msg.y:.0f}", f"theta:{msg.theta:.2f}")

    def onReceiveSpeed(self, topic_name, msg, timestamp):
        """Callback d'un subscriber ecal. Actualise la vitesse du robot"""
        self.speed = Speed.from_proto(msg)
    
    
    def onReceiveMatchStarted (self, topic_name, msg, timestamp):
        self.tempsDebutMatch = time.time()
        ecal_core.log_message("Match started at " + str(self.tempsDebutMatch))


### NAVIGATION ###
    def initNav(self):
        """ Initialise la navigation """
        self.nav.initialisation()

    def goToWaypoint(self,waypoint, theta: None | float = None ):
        """ Le robot va directement à un waypoint avec ou sans angle donné
        \nArgs, string:waypoint, float|None:theta"""
        if theta is None :
            theta = self.pos.theta
            #print(theta*180/pi)
        x,y = self.nav.getCoords(waypoint)
        self.setTargetPos(Pos(x,y,theta))
        #closest = self.nav.closestWaypoint(self.pos.x,self.pos.y)
        #self.pathFinder(closest,waypoint)

    def resetPosFromNav(self, waypoint, theta=None):
        print("Reseted nav at :", waypoint)
        if theta is None:
            theta = self.pos.theta
        x,y = self.nav.getCoords(waypoint)
        self.resetPos(Pos(x, y, theta))

    def pathFinder(self,dest,orientation):
        """Recherche le plus court chemin entre deux points.
        \nRetenu dans l'object self.nav.chemin
        \nUtiliser les noms des waypoints de graph.txt"""

        self.nav.entree = self.nav.closestWaypoint(self.pos.x,self.pos.y)
        self.nav.sortie = dest
        nav_pos = self.nav.findPath(self.pos.theta,orientation)

        self.n_points = len(self.nav.chemin)
        print(f"entree: {self.nav.entree}")
        self.current_point_index = 0
        self.nav.current = self.nav.chemin[self.current_point_index]
        print("Path found : ",self.nav.chemin)
        self.nav_pos = [Pos(p[0],p[1],p[2]) for p in nav_pos]
        #print("Pos's are : ",self.nav_pos)

        
    
    def isNavDestReached(self):
        """Si le dernier point de Nav est atteint renvoie True"""
        return self.nav_pos == []
    
    def onLidar (self, topic_name, msg, timestamp):
        self.lidar_pos = Pos.from_proto(msg)
    
    def recallageLidar (self):
        self.pos.x = self.lidar_pos.x
        self.pos.y = self.lidar_pos.y
        
    


    ### Actionneur ###
    def setActionneur(self, actionneur: Actionneur,val : ValeurActionneur | int):
        """ Définir en externe les valeurs à prendre 
        \nArgs, Actionneur:actionneur, ValeurActionneur|int:valeur
        \n Ex: Faire setActionneur(Actionneur.AxL,ValeurActionneur.UpAxL) pour piloter l'ax de gauche !"""
        if type(val) == int :
            msg = robot_pb.IO(id = actionneur.value , val = val)    
        else :
            msg = robot_pb.IO(id = actionneur.value , val = val.value)
        
        self.IO_pub.send(msg)

    def initActionneur(self):
        """Passage de tout les actionneurs à leur position de début de match \n bloquant pendant 1 sec"""
        time.sleep(0.1)
        self.setActionneur(Actionneur.Pince1,ValeurActionneur.OpenPince1)
        time.sleep(0.1)
        self.setActionneur(Actionneur.Pince2,ValeurActionneur.OpenPince2)
        time.sleep(0.1)
        self.setActionneur(Actionneur.Pince3,ValeurActionneur.OpenPince3)
        time.sleep(0.1)
        self.setActionneur(Actionneur.Pince4,ValeurActionneur.OpenPince4)
        time.sleep(0.1)
        self.setActionneur(Actionneur.Bras,ValeurActionneur.UpBras)
        time.sleep(0.1)
        self.setActionneur(Actionneur.Pano,ValeurActionneur.InitPano)
        time.sleep(0.1)
        self.setActionneur(Actionneur.AxBabord,ValeurActionneur.UpAxBabord)
        time.sleep(0.1)
        self.setActionneur(Actionneur.AxTribord,ValeurActionneur.UpAxTribord)
        time.sleep(0.1)

    def aruco(self, topic_name, msg, timestamp):
        """Callback Ecal du code getAruco, stocke la commande du panneau"""
        # print(msg)
        
        self.aruco_theta = msg.theta
        # position du centre de rotation
        self.aruco_y = msg.x - cos(np.deg2rad(self.aruco_theta)) * 15 
        self.aruco_x = -(msg.z - PANO_OFFSET) - sin(np.deg2rad(self.aruco_theta)) * 15
        self.aruco_time = time.time()
        #print("aruco : ",self.aruco_x,self.aruco_y)
        commande_pano = self.aruco_theta + self.pano_angle
        #print(f"aruco cmd : x = {self.aruco_x}\t y = {self.aruco_y}")
        if commande_pano > 180 : 
            commande_pano  = commande_pano - 360

        if commande_pano < -180 : 
           commande_pano  = commande_pano + 360

        self.commande_pano = commande_pano*PANO_CONVERT + ValeurActionneur.InitPano.value
    
    def panoDo(self,commande):
        """ensemble d'instruction bloquantes pour la procédure des Paneau solaires
        \nArgs: int:consigne du servo"""
        time.sleep(0.5)
        self.setActionneur(Actionneur.Bras,ValeurActionneur.DownBras)
        time.sleep(1)
        self.setActionneur(Actionneur.Pano,int(commande))
        #print("commande: ",commande)
        time.sleep(1)# il faut un sleep là sinon le robot bouge avec le pano encore en bas
        self.setActionneur(Actionneur.Bras,ValeurActionneur.UpBras)
        time.sleep(0.1)
        self.setActionneur(Actionneur.Pano,ValeurActionneur.InitPano)
        time.sleep(0.1)
    
    def set_strat(self, strat):
        self.strat = strat

    def set_pid_gain(self, gain, value):
        self._pid_gains[gain] = value
        kp, ki, kd = self._pid_gains
        msg = base_pb.MotorPid(motor_no=0, kp=kp, ki=ki, kd=kd)
        self.pid_pub.send(msg)
    

    def detection(self, topic_name, msg, timestamp):
        """ Try to find ennemies 
        \nSend 3 detected object Pos on ecal to visualize but saves all of them """
        def filter_pos(pos_size):
            pos, size = pos_size
            if 0 < pos.x < 3000 and  0 < pos.y < 2000 :# exclude object out of the table
                if sqrt(pos.x**2 + pos.y**2) < 1500 and size < 30 : # exclude close and tiny object ( test in conditions !)
                    return False
                return True

        amalgames = [(Pos(x,y,0).from_frame(self.pos), size) for x,y,size in zip(msg.x,msg.y,msg.size)]
        
        self.obstacles = list(filter(filter_pos,amalgames))
        for i,ob in enumerate(self.obstacles) :
            if i < 3:
                self.objects_pubs[i].send(ob[0].to_proto())
    

    def obstacle_in_way(self, target_pos: Pos):
        """Return True if obstacles in bounds towards target_pos"""
        if self.pos.distance(target_pos) < 1:
            return False
        dir = atan2(target_pos.y-self.pos.y,target_pos.x-self.pos.x)
        traj_frame = Pos(self.pos.x, self.pos.y, dir)

        for obj_pos, _size in self.obstacles:
            obj = obj_pos.to_frame(traj_frame)
            
            x_min, x_max, y_min, y_max = BOUNDS
            if x_min < obj.x < x_max and  y_min < obj.y < y_max :
                return True
            
        return False
    



    # def obstacle_dist_on_path(self,target_pos:Pos):
    #     """Return True if the robot could colide with something on path and cooord of problem"""
    #     if self.pos.distance(target_pos) < 1:
    #         return False, 3600
        
    #     problems = []
    #     for obj_pos,size in self.obstacles:
    #         dist, pt = dist_to_line(obj_pos, self.pos, target_pos)
    #         if dist < AVOIDANCE_OBSTACLE_MARGIN:
    #             problems.append(pt)
        
    #     if problems :
    #         closest_problem = min(problems, key=lambda x: self.pos.distance(x))
    #         return True, self.pos.distance(closest_problem)
    #     else:
    #         return False, 3600





if __name__ == "__main__":
    r = Robot()
    while(True):
        print(r.pos)
        time.sleep(0.5)
        
