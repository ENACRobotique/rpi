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
from IO.actionneurs import * 
import common
from common import Pos, Speed, dist_to_line, next_path, normalize_angle
import locomotion
import musics
from camera.conserveVisu import visuConserve
import random as rd
from scipy.stats import linregress

from enum import Enum
from dataclasses import dataclass
import numpy as np
import sw.nav.nav as nav 

import lcd_client as lcd
HEIGHT = 2000
WIDTH = 3000
DELTA = 40
XY_ACCURACY = 15  # mm
THETA_ACCURACY = radians(10) # radians
AVOIDANCE_OBSTACLE_MARGIN = 500 #in mm.  Standard robot enemy radius is 22 cm


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

class Robot:
    """Classe dont le but est de se subscribe à ecal pour avoir une représentation de l'état du robot
    
    Créez un objet de cette classe si vous avez besoin de connaître l'état du robot."""
    def __init__(self, name="robotStateHolder"):
        """x et y sont en mètres
        theta est en radian"""
        ecal_core.initialize(sys.argv, name)
        self.logger = logging.getLogger(name)
        logging.basicConfig(filename=next_path("/home/robot/logs/strat_log_{}.log"), level=logging.INFO)
        logging.getLogger().addHandler(logging.StreamHandler(sys.stdout))

        self.pos = Pos(0, 0, 0)
        self.last_d = 20 # ne pas mettre trop grand
        self.nb_pos_received = 0
        self.speed = Speed(0, 0, 0)
        self.last_target = Pos(0, 0, 0)
        self.nav = nav.Nav()
        self.command_sent = False
        self.vl53_data: dict[Actionneur,None|tuple] = {Actionneur.AimantBasDroit: None,
                           Actionneur.AimantBasGauche: None
                           }
        
        self.color = Team.AUCUNE
        self.tirette = Tirette.OUT
        self.strat = Strat.Audacieuse
        self.score = 0
        self.obstacles = []

        self._pid_gains = [0, 0, 0]     # Just for manual setting of PIDS

        self.actionneurs = IO_Manager()
        self.locomotion = locomotion.Locomotion()
        self.locomotion.start()
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
        calibration_menu = lcd.Menu("Calibrations", m)
        pid_page = lcd.Menu("PID", m)
        m.add_subpages(strat_choices_page, self.status_page, self.pos_page, self.score_page, pid_page, calibration_menu)

        self.calibration_lift_state = lcd.Text("State", calibration_menu, str(self.actionneurs.liftCalibrated))
        calibration_lift_choice = lcd.Choice("Lifts", calibration_menu, ["Push to cal"], self.calibrateLift)
        calibration_menu.add_subpages(calibration_lift_choice, self.calibration_lift_state)

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
        
        self.amalgame_sub = ProtoSubscriber("amalgames", lidar_pb.Amalgames)
        self.amalgame_sub.set_callback(self.detection)

        # When Using Robokontrol
        self.setPositionSub = ProtoSubscriber("set_position", robot_pb.Position)
        self.setPositionSub.set_callback(self.onSetTargetPostition)

        #self.proximitySub = ProtoSubscriber("proximity_status",lidar_pb.Proximity)
        #self.proximitySub.set_callback(self.onProximityStatus)


        self.vl53_0_sub = ProtoSubscriber("vl53_0",lidar_pb.Lidar)
        self.vl53_0_sub.set_callback(lambda topic_name, msg, timestamp : self.on_vl53(Actionneur.AimantBasGauche, topic_name, msg, timestamp))
        self.vl53_1_sub = ProtoSubscriber("vl53_1",lidar_pb.Lidar)
        self.vl53_1_sub.set_callback(lambda topic_name, msg, timestamp : self.on_vl53(Actionneur.AimantBasDroit, topic_name, msg, timestamp))
        # self.vl53_3_sub = ProtoSubscriber("vl53_3",lidar_pb.Lidar)
        # self.vl53_3_sub.set_callback(lambda topic_name, msg, timestamp : self.vl53_detect_plante(msg, Actionneur.Pince3))
        # self.vl53_4_sub = ProtoSubscriber("vl53_4",lidar_pb.Lidar)
        # self.vl53_4_sub.set_callback(lambda topic_name, msg, timestamp : self.vl53_detect_plante(msg, Actionneur.Pince4))
        
        ### PUB ECAL ###
        self.reset_pos_pub = ProtoPublisher("reset", robot_pb.Position)

        # self.IO_pub = ProtoPublisher("Actionneur",robot_pb.IO)

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
        self.actionneurs.initActionneur()

    def __repr__(self) -> str:
        return "Robot Enac status storage structure"

# ---------------------------- #
#             IHM              #
# ____________________________ #

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



    def on_lcd_event(self, event):
        if event.button == robot_pb.LCDEvent.Button.COLOR and event.value == 1:
            color = Team.BLEU if self.color == Team.JAUNE else Team.JAUNE
            self.set_color(color)
        elif event.button == robot_pb.LCDEvent.Button.TIRETTE:
            self.tirette = Tirette(event.value)

    
    def on_lcd_state(self, state):
        self.tirette = Tirette(state.tirette)

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

    def onReceiveMatchStarted (self, topic_name, msg, timestamp):
        self.tempsDebutMatch = time.time()
        ecal_core.log_message("Match started at " + str(self.tempsDebutMatch))

    def set_strat(self, strat):
        self.strat = strat

    def shuffle_play(self):
        for music in musics.playlist :
            self.play_music(music)

    def play_music(self, music):
        """Joue une musique"""
        for tone, duration in music:
            self.buzz(tone)
            time.sleep(duration)
    
    def calibrateLift(self, dummy):
        self.actionneurs.calibrateLift()
        self.calibration_lift_state.set_text(f"Lifts Cal:",f"{self.actionneurs.liftCalibrated}")
        self.lcd.set_page(self.calibration_lift_state)
    
    def ready_to_go(self):
        """ Rassembler les conditions nécéssaires pour que le robot commence son match"""
        a = self.color != Team.AUCUNE
        b = self.tirette == Tirette.OUT
        return a and b
# ---------------------------- #
#           CONTROL            #
# ____________________________ #
    
    def hasReachedTarget(self):
        return self.locomotion.hasReachedTarget()
        # d=sqrt((self.pos.x-self.last_target.x)**2 + (self.pos.y-self.last_target.y)**2)
        # hrt = (d <= XY_ACCURACY) and (abs(self.pos.theta - self.last_target.theta) <= THETA_ACCURACY)
        # return hrt 

    def setTargetPos(self, pos: Pos, frame=Frame.TABLE,blocking=False, timeout = 10):
        """Faire setTargetPos(Pos(x,y,theta)) en mm et angle en radian """

        if frame == Frame.ROBOTCENTRIC:
            pos += self.pos
        elif frame == Frame.ROBOT:
            pos = pos.from_frame(self.pos)
        pos.theta = normalize_angle(pos.theta)
        self.locomotion.set_target_pos(pos)
        self.last_target = pos
        
        if blocking :
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.hasReachedTarget():
                    return True
                time.sleep(0.1)
            return False

    def move(self, distance, direction, speed, blocking=False, timeout = 10):
        """
        avance de distance dans la direction direction, repère robot 
        """
        frame_pince = Pos(0, 0, direction)
        target = Pos(distance, 0, -direction).from_frame(frame_pince)
        self.locomotion.set_move_speed(speed)
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

    def set_speed(self, speed: Speed, duration=1):
        """
        Set robot speed in robot reference frame.
        Useful e.g. to move until a sensor triggers
        """
        self.locomotion.set_speed(speed, duration)
    
    def resetPos(self, position: Pos, timeout=2):
        self.logger.info(f"Reseting position to: {position} ")
        self.reset_pos_pub.send(position.to_proto())
        self.locomotion.reset_pos(position)
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
    

    def set_pid_gain(self, gain, value):
        self._pid_gains[gain] = value
        kp, ki, kd = self._pid_gains
        msg = base_pb.MotorPid(motor_no=0, kp=kp, ki=ki, kd=kd)
        self.pid_pub.send(msg)

# ---------------------------- #
#          NAVIGATION          #
# ____________________________ #   

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
        """Si le dernier point de Nav est atteint renvoie True\n
        Nécéssite de vider continuement la liste des points de nav !"""
        return self.nav_pos == []
        
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

# ---------------------------- #
#              IO              #
# ____________________________ #


    def detect_one_conserve(self, actionneur):
        if self.vl53_data[actionneur] is None:
            return None
        
        def idx(x, y):
            #return (7 - y) * 8 + (7 - x)
            return (8*x+(7-y))
        
        x_dist = [0 for _ in range (8)]
        for x in range (8):
            for y in range(2,8):
                x_dist[x]+= (self.vl53_data[actionneur])[idx(x,y)]
        
        min_x = min(enumerate(x_dist), key=lambda x: x[1])
        return min_x[0], min_x[1]/6
        
    def detect_best_conserve(self, actionneur):
        if self.vl53_data[actionneur] is None:
            return None
        
        def idx(x, y):
            #return (7 - y) * 8 + (7 - x)
            return (8*x+(7-y))
        
        def find_min_loc(tab):
            min_loc = []
            n = len(tab)

            for i in range(n):
                if (i == 0 or tab[i] < tab[i-1]) and (i == n-1 or tab[i] < tab[i+1]):
                    min_loc.append((i,tab[i]))

            return min_loc


        def remove_same_min(mini):
            for (ind1,val1) in mini:
                for (ind2, val2) in mini:
                    if ind1 == ind2 + 2:
                        mini.remove(min((ind1,val1),(ind2,val2), key=lambda a : a[0]))
            return mini                    

        def select_best_conserve(mini):
            min_val = min(mini, key=lambda x: x[1])[1]
            for ind, val in mini:
                if val > min_val + 600:
                    mini.remove((ind,val))
            dist2_center = [abs(3.5-ind) for (ind,val) in mini]
            ind_min = min(enumerate(dist2_center), key=lambda x: x[1])[0]
            return mini[ind_min]

        y_cons = [0 for _ in range (8)]
        for x in range (8):
            for y in range(2,8):
                y_cons[x]+= (self.vl53_data[actionneur])[idx(x,y)]

        min_loc = find_min_loc(y_cons)
        min_loc = remove_same_min(min_loc)
        indice_x,distance_capteur_conserve = select_best_conserve(min_loc) 
        return indice_x, distance_capteur_conserve/6
    

    def vl53_planche(self, actionneur):
        def curved_linspace_sin(base, amplitude, num):
            """
            Génère num points variant selon base + amplitude * (1 - cos(theta)) / 2
            où theta va de 0 à 2π.
            Résultat : de base -> base+amplitude -> base.
            """
            theta = np.linspace(0, 2*np.pi, num, endpoint=True)
            # (1 - cosθ)/2 fait une bosse normalisée de 0 à 1 puis à 0
            bump = (1 - np.cos(theta)) / 2
            return base + amplitude * bump
        
        pixel_angles = np.array([(i-3.5)*radians(45)/8 for i in range(8)])
        distortion = curved_linspace_sin(1, -0.05, 8)
        distance_matrix = np.empty((8,8))
        def idx(x, y):
            return (8*x+(7-y))
        
        for y in range(8):
            for x in range(8):
                distance_matrix[y,x] = self.vl53_data[actionneur][idx(x,y)]

        def reg_line(dists):
            dists = dists * distortion
            xs = dists * np.sin(pixel_angles)
            ys = dists * np.cos(pixel_angles)
            return linregress(xs, ys)
        
        lin1 = [(y, *reg_line(distance_matrix[y])) for y in range(8)]

        y, slope_best, _, r_best, _, stderr = min(lin1, key=lambda l: abs(l[5]))

        angle = np.arctan2(slope_best, 1)

        dist = np.mean(distance_matrix[y])

        #print(f"{y} -> {degrees(angle):+03.0f} : {slope_best:+05.2f} : {r_best:.2f}  sdterr:{stderr:.2f}")
        return angle, dist, r_best, stderr

    def vl53_planches2(self):
        if self.vl53_data[Actionneur.AimantBasGauche] is None or self.vl53_data[Actionneur.AimantBasDroit] is None:
            return None
        angle1, dist1, _, stderr1 = self.vl53_planche(Actionneur.AimantBasGauche)
        angle2, dist2, _, stderr2 = self.vl53_planche(Actionneur.AimantBasDroit)
        w1 = 1 / stderr1**2
        w2 = 1 / stderr2**2
        angle = (w1 * angle1 + w2 * angle2) / (w1 + w2)
        dist = (w1 * dist1 + w2 * dist2) / (w1 + w2)
        stderr = (w1 * stderr1 + w2 * stderr2) / (w1 + w2)
        #print(f"{degrees(angle):+03.0f} : {dist:+05.2f} : sdterr:{stderr:.2f}")
        return angle, dist, stderr

    def on_vl53(self, actionneur, topic_name, msg, timestamp):
        self.vl53_data[actionneur] = list(msg.distances)


if __name__ == "__main__":
    r = Robot()
    while(True):
        r.logger.info(r.pos)
        time.sleep(0.5)
        
