#!/usr/bin/env python3
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Publisher as ProtoPublisher
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.common.core import ReceiveCallbackData
from ecal.msg.string.core import Publisher as StringPublisher
import time
from math import sqrt, atan2, radians
import sys
import logging
import generated.common_pb2 as common_pb
import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb
import generated.messages_pb2 as base_pb
from IO.actionneurs import * 
from common import Pos, Speed, next_path, normalize_angle
import locomotion

from camera.conserveVisu import visuConserve
from scipy.stats import linregress

from enum import Enum
import numpy as np
import sw.nav.nav as nav 


HEIGHT = 2000
WIDTH = 3000
DELTA = 40
XY_ACCURACY = 15  # mm
THETA_ACCURACY = radians(10) # radians
#AVOIDANCE_OBSTACLE_MARGIN = 500 #in mm.  Standard robot enemy radius is 22 cm


# avoidance bounds
BOUNDS = (-250,500,-250,250)
ROTATE_BOUNDS = (-250,250,-250,250) # bounds for rotation
#BOUNDS = (-400,400,-400,400)
TABLE_BOUNDS = (0,3000,0,2000)
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
        if not ecal_core.is_initialized():
            ecal_core.initialize(name)
        self.logger = logging.getLogger(name)
        logging.basicConfig(filename=next_path("/home/robot/logs/strat_log_{}.log"), level=logging.INFO)
        logging.getLogger().addHandler(logging.StreamHandler(sys.stdout))

        self.pos = Pos(0, 0, 0)
        self.last_d = 20 # ne pas mettre trop grand
        self.nb_pos_received = 0
        self.speed = Speed(0, 0, 0)
        self.last_target = Pos(0, 0, 0)
        self.nav = nav.Nav() #Nav
        self.command_sent = False
        self.vl53_data: dict[Actionneur,None|tuple] = {Actionneur.AimantBasDroit: None,
                           Actionneur.AimantBasGauche: None
                           }
        
        self.color = Team.AUCUNE
        self.tirette = Tirette.OUT
        self.strat = Strat.Basique
        self.score = 0
        self.obstacles = [] # obstacles in table frame

        self._pid_gains = [0, 0, 0]     # Just for manual setting of PIDS

        self.actionneurs = IO_Manager()
        self.locomotion = locomotion.Locomotion()
        self.locomotion.start()

        ### SUB ECAL ###

        self.positionReportSub = ProtoSubscriber(common_pb.Position, "odom_pos")
        self.positionReportSub.set_receive_callback(self.onReceivePosition)

        self.speedReportSub = ProtoSubscriber(common_pb.Speed, "odom_speed")
        self.speedReportSub.set_receive_callback(self.onReceiveSpeed)

        self.amalgames_sub = ProtoSubscriber(lidar_pb.Amalgames, "amalgames")
        self.amalgames_sub.set_receive_callback(self.detection)

        self.balises_sub = ProtoSubscriber(lidar_pb.Balises, "balises_near_odom")
        self.balises_sub.set_receive_callback(self.on_detected_beacons)

        # When Using Robokontrol
        self.setPositionSub = ProtoSubscriber(common_pb.Position, "set_position")
        self.setPositionSub.set_receive_callback(self.onSetTargetPostition)

        #self.proximitySub = ProtoSubscriber(lidar_pb.Proximity, "proximity_status")
        #self.proximitySub.set_receive_callback(self.onProximityStatus)


        self.vl53_0_sub = ProtoSubscriber(lidar_pb.Lidar, "vl53_0")
        self.vl53_0_sub.set_receive_callback(lambda pub_id, data: self.on_vl53(Actionneur.AimantBasGauche, pub_id, data))
        self.vl53_1_sub = ProtoSubscriber(lidar_pb.Lidar, "vl53_1")
        self.vl53_1_sub.set_receive_callback(lambda pub_id, data: self.on_vl53(Actionneur.AimantBasDroit, pub_id, data))

        ### PUB ECAL ###
        self.reset_pos_pub = ProtoPublisher(common_pb.Position, "reset")

        # self.IO_pub = ProtoPublisher("Actionneur",robot_pb.IO)

        self.color_pub = ProtoPublisher(robot_pb.Side, "color")

        self.pid_pub = ProtoPublisher(base_pb.MotorPid, "pid_gains")


        self.logs_pub = StringPublisher("logs")
        self.objects_pubs = [ProtoPublisher(common_pb.Position, f"Obstacle{i}") for i in range(3)]
        time.sleep(1)

        self.nav.initialisation()
        self.folowingPath = False
        self.actionneurs.initActionneur()

    def __repr__(self) -> str:
        return "Robot Enac status storage structure"
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        self.positionReportSub.remove_receive_callback()
        self.speedReportSub.remove_receive_callback()
        self.amalgames_sub.remove_receive_callback()
        self.balises_sub.remove_receive_callback()
        self.setPositionSub.remove_receive_callback()
        self.vl53_0_sub.remove_receive_callback()
        self.vl53_1_sub.remove_receive_callback()

    def log(self, message:str):
        self.logger.info(message)
        self.logs_pub.send(message)
# ---------------------------- #
#             IHM              #
# ____________________________ #

    def set_color(self, c):
        self.color = c
        msg = robot_pb.Side()
        if self.color == Team.JAUNE:
            msg.color = robot_pb.Side.Color.YELLOW
        else:
            msg.color = robot_pb.Side.Color.BLUE
        self.logger.info(f"Equipe : {c}")
        self.color_pub.send(msg)


    def updateScore(self,points):

        self.score += points
        self.score_page.set_text(f"Score",f"{self.score}")

    
 
    def set_strat(self, strat):
        self.strat = strat

    
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
        self.locomotion.go_to(pos)
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
        BLOQUANT\n
        avance de distance dans la direction direction, repère robot 
        """
        frame_pince = Pos(0, 0, direction)
        target = Pos(distance, 0, -direction).from_frame(frame_pince)
        self.locomotion.set_move_speed(speed)
        return self.setTargetPos(target, Frame.ROBOT,blocking, timeout)
    
    
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
        self.log(f"Reseting position to: {position} ")
        self.reset_pos_pub.send(position.to_proto())
        self.locomotion.reset_pos(position)
        last_time = time.time()
        while True:
            if self.pos.distance(position) < 1 and abs(self.pos.theta - position.theta) < radians(1):
                self.log(f"Pos reseted to : {position.x},\t{position.y}, \t{position.theta} ")
                #self.pos = position
                break
            if time.time() - last_time > 1:
                self.log("reset_again")
                self.reset_pos_pub.send(position.to_proto())
                last_time = time.time()
            time.sleep(0.1)

    def resetPosNonBlocking(self,position:Pos):
        self.log(f"Reseting position to: {position} ")
        self.reset_pos_pub.send(position.to_proto())
        self.locomotion.reset_pos(position)
        
    
    def onSetTargetPostition (self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[common_pb.Position]):
        """Callback d'un subscriber ecal. Actualise le dernier ordre de position"""
        self.last_target = Pos.from_proto(data.message)

    def onReceivePosition (self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[common_pb.Position]):
        """Callback d'un subscriber ecal. Actualise la position du robot"""
        msg = data.message
        self.pos = Pos.from_proto(msg)
        self.nb_pos_received += 1
        self.pos_page.set_text(f"x:{msg.x:.0f} y:{msg.y:.0f}", f"theta:{msg.theta:.2f}")

    def onReceiveSpeed(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[common_pb.Speed]):
        """Callback d'un subscriber ecal. Actualise la vitesse du robot"""
        self.speed = Speed.from_proto(data.message)
    

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
            #self.log(theta*180/pi)
        x,y = self.nav.getCoords(waypoint)
        return self.setTargetPos(Pos(x,y,theta))
        #closest = self.nav.closestWaypoint(self.pos.x,self.pos.y)
        #self.pathFinder(closest,waypoint)

    def resetPosFromNav(self, waypoint, theta=None):
        self.log("Reseted nav at : {waypoint}")
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
        self.log(f"entree: {self.nav.entree}")
        self.current_point_index = 0
        self.nav.current = self.nav.chemin[self.current_point_index]
        self.log(f"Path found : {self.nav.chemin}")
        self.nav_pos = [Pos(p[0],p[1],p[2]) for p in nav_pos]
        #self.log("Pos's are : ",self.nav_pos)
        self.folowingPath = True
    
    def closeToNavPoint(self, nav_id):
        d=sqrt((self.pos.x-self.nav_pos[nav_id].x)**2 + (self.pos.y-self.nav_pos[nav_id].y)**2)
        return (d <= XY_ACCURACY)
    
    def isNavDestReached(self):
        """Si le dernier point de Nav est atteint renvoie True\n
        Nécéssite de vider continuement la liste des points de nav !"""
        end = self.closeToNavPoint(-1) and self.hasReachedTarget()
        if end:
            self.folowingPath = False
        return end 
    
    def on_detected_beacons(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[lidar_pb.Balises]):
        msg = data.message
        if self.lcd.current_page == self.beacons_page:
            nb_beacons_detected = len(msg.index)
            self.beacons_updates += 1
            self.beacons_page.set_text(f"Balises", f"{nb_beacons_detected} beacons {self.beacons_updates}")
        
    def detection(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[lidar_pb.Amalgames]):
        msg = data.message
        """ Try to find ennemies 
        \nSend 3 detected object Pos on ecal to visualize but saves all of them """
        def filter_pos(pos_size):
            
            pos, size = pos_size
            if 0+DELTA < pos.x < WIDTH-DELTA and  0+DELTA < pos.y < HEIGHT-DELTA :# exclude object out of the table
                if self.pos.distance(pos) < 1500 and size < 30 : # exclude close and tiny object ( test in conditions !)
                    return False
                if self.pos.distance(pos) < 150: # exclude object mixed up with the robot
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
    
    def obstacle_in_speed(self, speed: Speed):
        """Return True if obstacles in bounds towards speed direction"""
        speed_norm = speed.xy_norm()
        if speed_norm > 1:
            bounds = BOUNDS
            #if speed_norm > 300: # if speed is too high, we adapt the bounds
            #    x_min, x_max, y_min, y_max = bounds
            #    coef = speed_norm / 300
            #    bounds = (x_min, x_max*coef, y_min, y_max)
        else:
            bounds = ROTATE_BOUNDS
        dir = atan2(speed.vy, speed.vx)
        traj_frame = Pos(self.pos.x, self.pos.y, dir)

        for obj_pos, _size in self.obstacles:
            obj = obj_pos.to_frame(traj_frame)
            x_min, x_max, y_min, y_max = bounds
            if x_min < obj.x < x_max and  y_min < obj.y < y_max :
                return True
        return False

# ---------------------------- #
#              IO              #
# ____________________________ #





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

    def on_vl53(self, actionneur, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[lidar_pb.Lidar]):
        self.vl53_data[actionneur] = list(data.message.distances)


if __name__ == "__main__":
    with Robot() as r:
        while(True):
            # r.logger.info(r.pos)
            time.sleep(0.5)
        
