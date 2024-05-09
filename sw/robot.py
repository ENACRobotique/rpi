#!/usr/bin/env python3
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
import time
from math import sqrt, pi, cos, sin, atan2, radians,degrees
import sys
import logging
import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb
import generated.messages_pb2 as base_pb
import common
from common import Pos, Speed, dist_to_line, next_path # tkt ça marche

import random as rd

from enum import Enum
from dataclasses import dataclass
import numpy as np
import nav 

import lcd_client as lcd
HEIGHT = 2000
WIDTH = 3000
DELTA = 40
XY_ACCURACY = 15  # mm
THETA_ACCURACY = radians(10) # radians
AVOIDANCE_OBSTACLE_MARGIN = 500 #in mm.  Standard robot enemy radius is 22 cm

THETA_PINCES_BABORD = radians(60)  #pinces babord repère robot 
THETA_PINCES_TRIBORD = radians(-60)

# avoidance bounds 
BOUNDS = (-100,400,-250,250)

#timing for actionneur movements
ACT_TIME = 0.5 # seconds

class Team(Enum):
    BLEU = 1
    JAUNE = 2
    AUCUNE = 10

class Tirette(Enum):
    IN = 1
    OUT = 0

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
    InitPano = 1400

    OpenPince1 = 1120
    OpenPince2 = 1900
    OpenPince3 = 850
    OpenPince4 = 1070
    
    ClosePince1 = 790
    ClosePince2 = 1550
    ClosePince3 = 1200
    ClosePince4 = 1315

    ClosePincePot1 = 830
    ClosePincePot2 = 1590
    ClosePincePot3 = 1150
    ClosePincePot4 = 1260
    

    DownBras = 1960
    UpBras = 970
    
    UpAxBabord = 1020
    UpAxTribord = 140

    MidAxBabord = 500
    MidAxTribord = 640   

    DownAxBabord = 90
    DownAxTribord = 1020


class Robot:
    """Classe dont le but est de se subscribe à ecal pour avoir une représentation de l'état du robot
    
    Créez un objet de cette classe si vous avez besoin de connaître l'état du robot."""
    def __init__(self, name="robotStateHolder"):
        """x et y sont en mètres
        theta est en radian"""
        ecal_core.initialize(sys.argv, name)
        self.logger = logging.getLogger(name)
        logging.basicConfig(filename=next_path("/home/pi/logs/strat_log_{}.log"), level=logging.INFO)
        logging.getLogger().addHandler(logging.StreamHandler(sys.stdout))

        self.pos = Pos(0, 0, 0)
        self.nb_pos_received = 0
        self.speed = Speed(0, 0, 0)
        self.last_target = Pos(0, 0, 0)
        self.nav = nav.Nav()
        self.pano_angle = 0
        self.aruco_time = 0
        self.command_sent = False
        self.lidar_pos = Pos(0,0,0)
        self.vl53_data: dict[Actionneur,None|tuple] = {Actionneur.Pince1: None,
                           Actionneur.Pince2: None,
                           Actionneur.Pince3: None,
                           Actionneur.Pince4: None
                           }

        self.aruco_y = 0
        self.aruco_x = 0
        self.aruco_theta = 0

        self.color = Team.AUCUNE
        self.tirette = Tirette.OUT
        self.strat = Strat.Audacieuse
        self.score = 0
        self.obstacles = []

        self._pid_gains = [0, 0, 0]     # Just for manual setting of PIDS

        self.solar_offset = 125 # Basic solar offset
        self.solar_ratio = 10

        #self.tirette = robot_pb.IHM.T_NONE
        #self.color = robot_pb.IHM.C_NONE
        #self.proximityStatus = None

        #self.lastCommandNumber = None
        #self.lastFinishedActionNumber = None
        #self.pointsEstimes =0

        self.tempsDebutMatch = None

        m=lcd.Menu("Robot", None)
        strat_choices_page = lcd.Choice("Strat", m, [s for s in Strat], self.set_strat)
        #detect_range_page = lcd.Number("Dist detection", m, 20, 150, None)
        self.pos_page = lcd.Text("Position", m, "---")
        self.status_page = lcd.Text("Status", m, "---")
        self.score_page = lcd.Text("Score", m, "0")
        pid_page = lcd.Menu("PID", m)
        solar_page = lcd.Menu("Solar",m)
        m.add_subpages(strat_choices_page, self.status_page, self.pos_page, self.score_page, pid_page, solar_page)

        kp_page = lcd.Number("Kp", pid_page, 0, 10, lambda x: self.set_pid_gain(0, x))
        ki_page = lcd.Number("Ki", pid_page, 0, 1, lambda x: self.set_pid_gain(1, x))
        kd_page = lcd.Number("Kd", pid_page, 0, 5, lambda x: self.set_pid_gain(2, x))
        pid_page.add_subpages(kp_page, ki_page, kd_page)

        solar_offset_page = lcd.Number("Offset", solar_page, 100, 140, self.set_solar_offset)
        solar_ratio_page = lcd.Number("Ratio", solar_page, 0, 2, self.set_solar_ratio)
        solar_page.add_subpages(solar_offset_page, solar_ratio_page)

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


        self.vl53_started = {Actionneur.Pince1: False, Actionneur.Pince2: False, Actionneur.Pince3: False, Actionneur.Pince4: False}
        self.vl53_1_sub = ProtoSubscriber("vl53_1",lidar_pb.Lidar)
        self.vl53_1_sub.set_callback(lambda topic_name, msg, timestamp : self.vl53_detect_plante(msg, Actionneur.Pince1))
        self.vl53_2_sub = ProtoSubscriber("vl53_2",lidar_pb.Lidar)
        self.vl53_2_sub.set_callback(lambda topic_name, msg, timestamp : self.vl53_detect_plante(msg, Actionneur.Pince2))
        self.vl53_3_sub = ProtoSubscriber("vl53_3",lidar_pb.Lidar)
        self.vl53_3_sub.set_callback(lambda topic_name, msg, timestamp : self.vl53_detect_plante(msg, Actionneur.Pince3))
        self.vl53_4_sub = ProtoSubscriber("vl53_4",lidar_pb.Lidar)
        self.vl53_4_sub.set_callback(lambda topic_name, msg, timestamp : self.vl53_detect_plante(msg, Actionneur.Pince4))
        
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
        self.logger.info(f"Equipe : {c}")
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
        #self.logger.info(f"go to: {pos}")
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
        """
        avance de distance dans la direction direction, repère robot 
        """
        frame_pince = Pos(0, 0, direction)
        target = Pos(distance, 0, -direction).from_frame(frame_pince)
        return self.setTargetPos(target, Frame.ROBOT,blocking, timeout)
    
    def move_rel(self,x,y,blocking=False, timeout = 10):
        if x : 
            self.move(sqrt(x**2+y**2),atan2(y,x),blocking, timeout)
        else :
            self.move(y,pi/2*np.sign(y),blocking, timeout)
    
    def heading(self,angle,blocking=False, timeout = 10):
        """ S'oriente vers la direction donnée
         \nArgs float:theta en radian""" 
        return self.setTargetPos(Pos(self.pos.x,self.pos.y,angle),Frame.TABLE, blocking, timeout)
    
    def rotate(self,angle,blocking=False, timeout = 10):
        """ Rotation en relatif
         \nArgs, float:theta en radians """
        self.heading(self.pos.theta + angle,blocking=False, timeout = 10)
    
    def resetPos(self, position: Pos, timeout=2):
        self.logger.info(f"Reseting position to: {position} ")
        self.reset_pos_pub.send(position.to_proto())
        last_time = time.time()
        while True:
            if self.pos.distance(position) < 1 and abs(self.pos.theta - position.theta) < radians(1):
                self.logger.info(f"Pos reseted to : {position.x},\t{position.y}, \t{position.theta} ")
                #self.pos = position
                break
            if time.time() - last_time > 1:
                self.logger.info("reset_again")
                self.reset_pos_pub.send(position.to_proto())
                last_time = time.time()
            time.sleep(0.1)
    
    def updateScore(self,points):
        self.buzz(ord('B'))
        time.sleep(0.1)
        self.buzz(ord('E')+7)
        time.sleep(0.1)
        #self.buzz(ord('0'))
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
        self.nb_pos_received += 1
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
            #self.logger.info(theta*180/pi)
        x,y = self.nav.getCoords(waypoint)
        return self.setTargetPos(Pos(x,y,theta))
        #closest = self.nav.closestWaypoint(self.pos.x,self.pos.y)
        #self.pathFinder(closest,waypoint)

    def resetPosFromNav(self, waypoint, theta=None):
        self.logger.info("Reseted nav at : {waypoint}")
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
        self.logger.info(f"entree: {self.nav.entree}")
        self.current_point_index = 0
        self.nav.current = self.nav.chemin[self.current_point_index]
        self.logger.info(f"Path found : {self.nav.chemin}")
        self.nav_pos = [Pos(p[0],p[1],p[2]) for p in nav_pos]
        #self.logger.info("Pos's are : ",self.nav_pos)

        
    
    def isNavDestReached(self):
        """Si le dernier point de Nav est atteint renvoie True"""
        return self.nav_pos == []
    
    def onLidar(self, topic_name, msg, timestamp):
        self.lidar_pos = Pos.from_proto(msg)
    
    def recallageLidar(self,tolerance, using_theta : bool = False):
        if using_theta:
            theta = self.lidar_pos.theta
        else:
            theta = self.pos.theta
        if self.pos.distance(self.lidar_pos) < tolerance :
            self.resetPos(Pos(self.lidar_pos.x,self.lidar_pos.y,theta))
        
    


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
        time.sleep(0.15)

    def initActionneur(self):
        """Passage de tout les actionneurs à leur position de début de match \n bloquant pendant 1 sec"""
        self.setActionneur(Actionneur.Pince1,ValeurActionneur.OpenPince1)
        self.setActionneur(Actionneur.Pince2,ValeurActionneur.OpenPince2)
        self.setActionneur(Actionneur.Pince3,ValeurActionneur.OpenPince3)
        self.setActionneur(Actionneur.Pince4,ValeurActionneur.OpenPince4)
        self.setActionneur(Actionneur.Bras,ValeurActionneur.UpBras)
        self.setActionneur(Actionneur.Pano,ValeurActionneur.InitPano)
        self.setActionneur(Actionneur.AxBabord,ValeurActionneur.UpAxBabord)
        self.setActionneur(Actionneur.AxTribord,ValeurActionneur.UpAxTribord)

    def set_solar_offset(self, x):
        self.solar_offset = x
    
    def set_solar_ratio(self,x):
        self.solar_ratio = x

    def aruco(self, topic_name, msg, timestamp):
        """Callback Ecal du code getAruco, stocke la commande du panneau"""
        # self.logger.info(msg)
        
        self.aruco_theta = msg.theta
        # position du centre de rotation
        self.aruco_y = msg.x - cos(np.deg2rad(self.aruco_theta)) * 15 
        self.aruco_x = -(msg.z - self.solar_offset) - sin(np.deg2rad(self.aruco_theta)) * 15
        self.aruco_time = time.time()
        #self.logger.info("aruco : ",self.aruco_x,self.aruco_y)
        commande_pano = self.aruco_theta + self.pano_angle
        #self.logger.info(f"aruco cmd : x = {self.aruco_x}\t y = {self.aruco_y}")
        if commande_pano > 180 : 
            commande_pano  = commande_pano - 360

        if commande_pano < -180 : 
           commande_pano  = commande_pano + 360

        self.commande_pano = - commande_pano*self.solar_ratio

    def commandeRoueSolaire(self,commande):
        self.setActionneur(Actionneur.Pano, int(commande + ValeurActionneur.InitPano.value))
    
    def panoDo(self,commande, precommande):
        """ensemble d'instruction bloquantes pour la procédure des Paneau solaires
        \nArgs: int:consigne du servo"""
        self.setActionneur(Actionneur.Bras,ValeurActionneur.DownBras)
        time.sleep(1)
        self.setActionneur(Actionneur.Pano,int(commande + ValeurActionneur.InitPano.value - precommande))
        #self.logger.info("commande: ",commande)
        time.sleep(0.5)# il faut un sleep là sinon le robot bouge avec le pano encore en bas
        self.setActionneur(Actionneur.Bras,ValeurActionneur.UpBras)
        self.setActionneur(Actionneur.Pano,ValeurActionneur.InitPano)
        
    
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
            if 0+DELTA < pos.x < WIDTH-DELTA and  0+DELTA < pos.y < HEIGHT-DELTA :# exclude object out of the table
                if sqrt((self.pos.x-pos.x)**2 + (self.pos.y-pos.y)**2) < 1500 and size < 30 : # exclude close and tiny object ( test in conditions !)
                    return False
                if sqrt((self.pos.x-pos.x)**2 + (self.pos.y-pos.y)**2) < 150: # exclude object mixed up with the robot
                    return False
                return True
            return False

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
    
    def vl53_detect_plante(self, msg, id):
        self.vl53_started[id] = True
        distances = list(msg.distances)
        #distance_matrix = np.empty((8,8))

        def idx(x, y):
            return (7 - y) * 8 + (7 - x)
        
        self.vl53_data[id] = None

        index_mins = []
        for y in range(3, 6):
            index_min = min(range(8), key=lambda x: distances[idx(x, y)])
            index_mins.append(index_min)
        index_min = sorted(index_mins)[len(index_mins)//2]
        dist = distances[idx(index_min, index_mins.index(index_min))]
        if dist < 300:
            self.vl53_data[id] = ((index_min - 3.5) * 5.625, dist)
        return

        
        for y in range(8):
            for x in range(8):
                distance_matrix[y,x] = distances[idx(x,y)]

        x_mins_lines = []
        ys = []
        for y in range(1,6):
            line = distance_matrix[y]
            x_mins_before = set()
            x_mins_after = {0,1,2,3,4,5,6,7}
            while x_mins_after != x_mins_before:
                x_mins_before = x_mins_after
                x_mins_after = set()
                for x in x_mins_before:
                    if x == 0:
                        x_mins_after.add(min([x,x+1], key = lambda a: line[a]))
                    elif x == 7:
                        x_mins_after.add(min([x-1,x], key = lambda a: line[a]))
                    else:
                        x_mins_after.add(min([x-1,x,x+1], key = lambda a: line[a]))
            
            to_remove = []
            for x in x_mins_after:
                if line[x] > 200:
                    to_remove.append(x)
            
            for x in to_remove:
                x_mins_after.remove(x)

            if 0 < len(x_mins_after) <= 2:
                ground = False
                if y == 5:
                    for x in x_mins_after:
                        if line[x] > 90: ground = True

                if ground == False:
                    x_mins_lines.append(x_mins_after)
                    ys.append(y)
        
        # self.logger.info("#############")
        # self.logger.info(ys)
        # self.logger.info(x_mins_lines)
        # for y,x in zip(ys,self.x_mins_lines) :
        #     self.logger.info(f"y={y} mins={x} val={[self.distance_matrix[y][a] for a in x]}")
        # for y in ys:
        #     self.logger.info(f"y={y} distances= {self.distance_matrix[y]}")


        if len(x_mins_lines) < 3:
            self.vl53_angle[id] = []
            self.vl53_distance[id] = []
            return
        
        single = []
        for i in range(len(x_mins_lines)):
            x = x_mins_lines[i]
            if len(x) == 1:
                single = [i]
        
        if single:
            # self.logger.info("Single plant")
            number_of_line = len(x_mins_lines)
            x_moy = x_mins_lines[single[0]].pop()
            distance_moy = distance_matrix[ys[single[0]]][x_moy]
            for i in range(number_of_line):
                if i != single[0]:
                    if i in single:
                        x = x_mins_lines[i].pop()
                        x_moy += x
                        distance_moy += distance_matrix[ys[i]][x]
            x_moy = x_moy / len(single)
            distance_moy = distance_moy / len(single)
            # self.logger.info(f"Position_x: {x_moy} Distance: {distance_moy}")
            self.vl53_angle[id] = [(x_moy - 3.5) * 5.625]
            self.vl53_distance[id] = [distance_moy]

        else:
            # self.logger.info("Two plant")
            number_of_line = len(x_mins_lines)
            xs_0 = list(x_mins_lines[0])
            xs_0.sort()
            x_moy_0 = xs_0[0]
            distance_moy_0 = distance_matrix[ys[0]][x_moy_0]
            x_moy_1 = xs_0[1]
            distance_moy_1 = distance_matrix[ys[0]][x_moy_1]
            for i in range(1,number_of_line):
                xs = list(x_mins_lines[i])
                xs.sort()
                x_moy_0 += xs[0]
                distance_moy_0 += distance_matrix[ys[0]][xs[0]]
                x_moy_1 += xs[1]
                distance_moy_1 += distance_matrix[ys[0]][xs[1]]
            x_moy_0 = x_moy_0 / number_of_line
            x_moy_1 = x_moy_1 / number_of_line
            distance_moy_0 = distance_moy_0 / number_of_line
            distance_moy_1 = distance_moy_1 / number_of_line
            # self.logger.info(f"Position_x: {x_moy_0} Distance: {distance_moy_0}")
            # self.logger.info(f"Position_x: {x_moy_1} Distance: {distance_moy_1}")
            if distance_moy_0 < distance_moy_1:
                self.vl53_angle[id] = [(x_moy_0 - 3.5) * 5.625, (x_moy_1 - 3.5) * 5.625]
                self.vl53_distance[id] = [distance_moy_0, distance_moy_1]
            else:
                self.vl53_angle[id] = [(x_moy_1 - 3.5) * 5.625, (x_moy_0 - 3.5) * 5.625]
                self.vl53_distance[id] = [distance_moy_1, distance_moy_0]


    
    def shuffle_play(self):
        i = rd.randint(1,3)
        if i==1:
            self.play_Space_oddity()
        elif i==2:
            self.play_Rick_Roll()
        elif i==3:
            self.play_rocket_man()
    
    def play_Space_oddity(self):
        """Lance la musique de Bowie avant le lancement """
        time.sleep(1)
        self.buzz(ord('G'))
        time.sleep(0.2)
        self.buzz(ord('G'))
        time.sleep(0.2)
        self.buzz(ord('C')+7) #
        time.sleep(0.3)
        self.buzz(ord('D')+7)
        time.sleep(0.3)
        self.buzz(ord('F')+7)
        time.sleep(0.3)
        self.buzz(ord('E')+7)
        time.sleep(0.35)
        self.buzz(ord('D')+7)
        time.sleep(0.35)
        self.buzz(ord('C')+7)
        time.sleep(0.35)
        self.buzz(ord('B')+7)
        self.buzz(ord('B')+7)
        self.buzz(ord('B')+7)
        time.sleep(1)
        self.buzz(ord('B')+7)
        time.sleep(0.35)
        self.buzz(ord('E')+7)
        time.sleep(0.35)
        self.buzz(ord('D')+7)
        time.sleep(0.35)
        self.buzz(ord('C')+7)
        time.sleep(0.35)
        self.buzz(ord('B')+7)
        time.sleep(0.35)
        self.buzz(ord('C')+7)
        time.sleep(0.35)
        self.buzz(ord('D')+7)
        time.sleep(0.2)
        self.buzz(ord('C')+7)
        time.sleep(0.2)
        self.buzz(ord('A')+7)

    
    def play_Rick_Roll(self):
        
        time.sleep(0.12)
        self.buzz(ord('A')) #Ne
        time.sleep(0.2)
        self.buzz(ord('B')) #Ver
        time.sleep(0.2)
        self.buzz(ord('D')) #Go
        time.sleep(0.2)
        self.buzz(ord('B')) #na
        time.sleep(0.3)
        self.buzz(ord('F')) #Give
        time.sleep(0.3)
        self.buzz(ord('F')) #You
        time.sleep(0.3)
        self.buzz(ord('E')) # Up
        time.sleep(0.5)

        self.buzz(ord('A')) #Ne 
        time.sleep(0.2)
        self.buzz(ord('B')) # Ver
        time.sleep(0.2)
        self.buzz(ord('C')) #Go
        time.sleep(0.2)
        self.buzz(ord('A')) # Na
        time.sleep(0.3)
        self.buzz(ord('E')) #Let
        time.sleep(0.3)
        self.buzz(ord('E')) #You
        time.sleep(0.3)
        self.buzz(ord('D')) # Do-
        time.sleep(0.3)
        self.buzz(ord('C')) # -oo-
        time.sleep(0.2)
        self.buzz(ord('B')) # -wn
        time.sleep(0.5)

        self.buzz(ord('A')) #Ne
        time.sleep(0.2)
        self.buzz(ord('B')) #Ver
        time.sleep(0.2)
        self.buzz(ord('D')) #Go
        time.sleep(0.2)
        self.buzz(ord('B')) #na
        time.sleep(0.3)
        self.buzz(ord('D')) #run 
        time.sleep(0.3)
        self.buzz(ord('E')) #arouund
        time.sleep(0.3)
        self.buzz(ord('C')) #
        time.sleep(0.3)
        self.buzz(ord('A')) # 
        time.sleep(0.15)
        self.buzz(ord('A')) #
        time.sleep(0.3)
        self.buzz(ord('E')) #
        time.sleep(0.45)
        self.buzz(ord('D')) #
    

def play_rocket_man(self):
        
        time.sleep(1)
        self.buzz(ord('F')) #and
        time.sleep(0.4)
        self.buzz(ord('F')) #I 

        self.buzz(ord('B')) #think  Bbemol normalement 
        time.sleep(0.4)
        self.buzz(ord('B')) #it's
        
        self.buzz(ord('D')) #go-
        time.sleep(0.4)
        self.buzz(ord('D')) #na 

        self.buzz(ord('E')) #Be 
        time.sleep(0.8)

        self.buzz(ord('D')) #a
        time.sleep(0.4)

        self.buzz(ord('C')) #long
        time.sleep(0.8)

        self.buzz(ord('D')) #long
        time.sleep(0.8)

        self.buzz(ord('B')) #time  Bbemol
        time.sleep(1.2)

        self.buzz(ord('G')) #l
        time.sleep(0.4)

        self.buzz(ord('B')) #   Bbemol
        time.sleep(0.8)

        self.buzz(ord('C')) #   
        time.sleep(0.8)

        self.buzz(ord('E'))  #  Ebemol normalement   
        time.sleep(0.8)

        self.buzz(ord('D'))  #  Ebemol normalement   
        time.sleep(0.4)

        self.buzz(ord('B'))  #  Bbemol normalement   
        time.sleep(0.4)

        self.buzz(ord('B'))  #  Bbemol normalement   
        time.sleep(0.4)

        self.buzz(ord('C'))  #  
        time.sleep(0.4)

        self.buzz(ord('D'))  #   
        time.sleep(0.4)

        self.buzz(ord('B'))  #  Bbemol normalement   
        time.sleep(0.4)

        

        




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
        self.logger.info(r.pos)
        time.sleep(0.5)
        
