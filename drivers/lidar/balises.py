from itertools import combinations
import ecal.core.core as ecal_core
import math
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
import numpy as np
import time, os, sys
from numpy import typing as npt
import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb
import itertools
from collections import namedtuple
RealBeacon = namedtuple('RealBeacon',['id','x','y'])
from scipy.optimize import least_squares

REAL_BEACONS_T = [[-94, 1000], [3094, 50], [3094, 1950]]
B0 = RealBeacon(0,  -94, 1000)
B1 = RealBeacon(1, 3094,   50)
B2 = RealBeacon(2, 3094, 1950)
# B3 = RealBeacon(3,    ?,    ?)
RealBeacons = [B0, B1, B2]
MAX_COST = 15000
TOLERANCE = 500 #mm
BEACON_MAX_SIZE = 150 # mm 
BEACON_MIN_SIZE = 50 # mm 
LOST = False
OK = True

def normalize_angle(a):
    while a > np.pi:
        a -= 2*np.pi
    while a < -np.pi:
        a+= 2*np.pi
    return a

class BeaconFinder:
    def __init__(self):
        self.odom_pos = (0, 0)
        ecal_core.initialize(sys.argv, "balises")

        # Subscribers
        self.sub_lidar = ProtoSubscriber("amalgames", lidar_pb.Amalgames)
        self.sub_lidar.set_callback(self.amalgames_cb)
        
        self.sub_odom = ProtoSubscriber("odom_pos", robot_pb.Position)
        self.sub_odom.set_callback(self.get_balises_odom)

        self.pub_lidar = ProtoPublisher("lidar_pos", robot_pb.Position)

        # Publishers
        self.pub_balises_odom = ProtoPublisher("balises_odom", lidar_pb.Balises)
        self.pub_closest_to_odom_beacons = ProtoPublisher("balises_near_odom", lidar_pb.Balises)

        self.initialized = False
        self.odomBeacon = [(0, 0),(0, 0),(0, 0)]#,(0, 0)]
        self.lastOdom = robot_pb.Position(x=0,y=0,theta=0)
        self.lidarPos = robot_pb.Position(x=500,y=500,theta=0)
        self.visibleBeacons = []
        self.lostBeacons = []
        self.estimatedBeacons = [(0, 0),(0, 0),(0, 0)]#,(0, 0)]
        self.predictions = [(0, 0),(0, 0),(0, 0)]#,(0, 0)]
        self.updated = False
        self.dx,self.dy,self.dtheta = 0,0,0
    def get_balises_odom(self, topic_name, msg, time):
        """ Calcule la position des balises à partir de l'odométrie\n"""
        xs = []
        ys = []
        ids = []
    
        for id, balise in enumerate(RealBeacons):
            dx = balise.x - msg.x
            dy = balise.y - msg.y
            x_b_rr = dx * np.cos(msg.theta) + dy * np.sin(msg.theta)
            y_b_rr = dy * np.cos(msg.theta) - dx * np.sin(msg.theta)
            ids.append(balise.id)
            xs.append(x_b_rr)
            ys.append(y_b_rr)
            self.odomBeacon[id] = x_b_rr,y_b_rr
        self.lastOdom = msg
        
        # Pour les afficher sur radarQT
        msg = lidar_pb.Balises(index=ids, x=xs, y=ys)
        self.pub_balises_odom.send(msg)
    
    @staticmethod
    def to_robot_frame(beacon: RealBeacon, posRobot : robot_pb.Position):
        """From table frame to robot frame"""
        dx = beacon.x - posRobot.x
        dy = beacon.y - posRobot.y
        x_b_rr = dx * np.cos(posRobot.theta) + dy * np.sin(posRobot.theta)
        y_b_rr = dy * np.cos(posRobot.theta) - dx * np.sin(posRobot.theta)
        return x_b_rr, y_b_rr

    def dist_2_pts(self,pt1, pt2):
        return math.sqrt((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2)
    
    def get3rd(self, knownIdA, knownIdB, unkwownIdC):
        radiusA = self.dist_2_pts(RealBeacons[knownIdA],RealBeacons[unkwownIdC])
        radiusB = self.dist_2_pts(RealBeacons[knownIdB],RealBeacons[unkwownIdC])
        pA = self.estimatedBeacons[knownIdA]
        pB = self.estimatedBeacons[knownIdB]
        xA, yA = pA
        xB, yB = pB
        print('x1,y1 ',xA,yA, ' x2,y2', xB,yB)

        d = self.dist_2_pts(pA, pB)

        if radiusA + radiusB < d or abs(radiusA - radiusB) > d:
            raise ValueError("No intersection points found. The circles do not intersect.")

        a = (radiusA**2 - radiusB**2 + d**2) / (2 * d)
        h = math.sqrt(radiusA**2 - a**2)

        xC = xA + a * (xB - xA) / d
        yC = yA + a * (yB - yA) / d

        intersection1 = (xC + h * (yB - yA) / d, yC - h * (xB - xA) / d)
        intersection2 = (xC - h * (yB - yA) / d, yC + h * (xB - xA) / d)


        return intersection1, intersection2
    
    def estimate_third_beacon(self, idxs, points):
        distance_01 = self.dist_2_pts(self.initialGuess[0], self.initialGuess[1])
        distance_02 = self.dist_2_pts(self.initialGuess[0], self.initialGuess[2])
        distance_12 = self.dist_2_pts(self.initialGuess[1], self.initialGuess[2])
        # print(idxs,distance_01,distance_02,distance_12)

        if 1 in idxs and 2 in idxs:  
            return self.estimate_third_from_two(points[0], points[1], distance_01, distance_02)

        elif 1 in idxs and 3 in idxs:  
            return self.estimate_third_from_two(points[0], points[1], distance_01, distance_12)

        elif 2 in idxs and 3 in idxs: 
            return self.estimate_third_from_two(points[0], points[1], distance_02, distance_12)
    
    def estimate_third_from_two(self, p1, p2, dist1, dist2):

        x1, y1 = p1
        x2, y2 = p2
        print('x1,y1 ',x1,y1, ' x2,y2', x2,y2)

        d = self.dist_2_pts(p1, p2)

        if dist1 + dist2 < d or abs(dist1 - dist2) > d:
            raise ValueError("No intersection points found. The circles do not intersect.")

        a = (dist1**2 - dist2**2 + d**2) / (2 * d)
        h = math.sqrt(dist1**2 - a**2)

        x3 = x1 + a * (x2 - x1) / d
        y3 = y1 + a * (y2 - y1) / d

        intersection1 = (x3 + h * (y2 - y1) / d, y3 - h * (x2 - x1) / d)
        intersection2 = (x3 - h * (y2 - y1) / d, y3 + h * (x2 - x1) / d)

        return intersection2
    
    def assign_beacons(self, amalgames):
        """Dans le cas où aucune balise n’est clairement identifiée, cette méthode teste toutes les combinaisons de 3 amalgames pour vérifier si leurs distances correspondent aux distances entre les balises théoriques. Si un triplet de points correspond (avec une certaine tolérance), il est considéré comme valide.
        C'est la méthode Jonathan si vraiment on s'en sort plus"""
        d_ab = self.dist_2_pts(self.initialGuess[0], self.initialGuess[1])  # Beacon A - Beacon B
        d_ac = self.dist_2_pts(self.initialGuess[0], self.initialGuess[2])  # Beacon A - Beacon C
        d_bc = self.dist_2_pts(self.initialGuess[1], self.initialGuess[2])  # Beacon B - Beacon C

        combinations_of_points = list(itertools.combinations(amalgames, 3))

        for comb in combinations_of_points:
            d_fg = self.dist_2_pts(comb[0], comb[1])  # Distance between F and G
            d_fh = self.dist_2_pts(comb[0], comb[2])  # Distance between F and H
            d_gh = self.dist_2_pts(comb[1], comb[2])  # Distance between G and H

            matching = self.check_distances([ d_ab, d_ac, d_bc],d_fg, d_fh, d_gh)

            if matching:
                print(f"Valid combination found: {comb}")
                assignment = self.get_assignment(comb, d_fg, d_fh, d_gh, d_ab, d_ac, d_bc)
                print(f"Assigned points: {assignment}")
                return  assignment
 
    def get_assignment(self, comb, d_fg, d_fh, d_gh, d_ab, d_ac, d_bc):

        assignment = {}
        distances = [(d_fg, 'AB'), (d_fh, 'AC'), (d_gh, 'BC')]

        distances.sort()  

        for i, (dist, beacon) in enumerate(distances):
            if beacon == 'AB':
                assignment['1'] = self.initialGuess[0]  # Beacon A
                assignment['2'] = self.initialGuess[1]  # Beacon B
            elif beacon == 'AC':
                assignment['1'] = self.initialGuess[0]  # Beacon A
                assignment['3'] = self.initialGuess[2]  # Beacon C
            elif beacon == 'BC':
                assignment['2'] = self.initialGuess[1]  # Beacon B
                assignment['3'] = self.initialGuess[2]  # Beacon C

        return assignment

    def check_distances(self, dist_abc, dist_12, dist_13, dist_23):

        threshold = 100  # Tolerance value (can be adjusted)
        dist_12_close = abs(dist_abc[0] - dist_12) < threshold
        dist_13_close = abs(dist_abc[1] - dist_13) < threshold
        dist_23_close = abs(dist_abc[2] - dist_23) < threshold

        return dist_12_close and dist_13_close and dist_23_close
    
    def closest_amalgamme(self, point, amalgames):
        """Returns index and dist of the closest amalgame in the list"""
        dists = [self.dist_2_pts(point, amalgame) for amalgame in amalgames]
        index = np.argmin(dists)
        return index, dists[index]

    def amalgames_cb(self, topic_name, msg, time):
        self.estimate_beacons_pos(msg)
        self.estimate(np.array(self.estimatedBeacons))

    def estimate_beacons_pos(self, msg):
        # On filtre les amalgames qui ne sont pas des balises
        amalgames = [(x, y) for (x, y, size) in zip(msg.x, msg.y, msg.size) if  size < BEACON_MAX_SIZE]
        
        #Méthode naïve
        self.visibleBeacons = []
        self.lostBeacons = []
        
        for beacon in RealBeacons:
            id = beacon.id
            last_estimation = self.to_robot_frame(beacon, self.lidarPos)# Replace lastOdom by la position lidar précédement calculée
            indexMin, dist = self.closest_amalgamme(last_estimation, amalgames)
            if dist <= TOLERANCE:
                self.visibleBeacons.append(id)
                self.estimatedBeacons[id] = amalgames[indexMin]
            else:
                self.lostBeacons.append(id)
                # on essai de le déduire de l'odom
                indexMin, dist = self.closest_amalgamme(self.odomBeacon[id], amalgames)
                if dist <= TOLERANCE:
                    self.estimatedBeacons[id] = amalgames[indexMin]
                    self.visibleBeacons.append(id)
                    print(f"Guessed {id}")

        if len(self.visibleBeacons) < 2:
            print("OSCOUR JE SUIS PERDU")
        if len(self.visibleBeacons) == 3:
            print("") # utile pour debug
            
        if len(self.visibleBeacons) > 0:
            self.updated = True
            # On ne publie que les balises trouvée
            ids = []
            xs = []
            ys = []
            for id in self.visibleBeacons:
                x,y = self.estimatedBeacons[id]
                ids.append(id)
                xs.append(x)
                ys.append(y)
            msg = lidar_pb.Balises(
                index=ids, x=xs, y=ys)
            self.pub_closest_to_odom_beacons.send(msg)
    
    def rotation_matrix(self,theta):
        return np.array([
            [np.cos(theta),-np.sin(theta)],
            [np.sin(theta),np.cos(theta)]
        ])
    
    def objective_function(self,pos_t, detected_beacons_r, real_beacons_t):
        theta=pos_t[0]
        tx,ty=pos_t[1],pos_t[2]
        R= self.rotation_matrix(theta)
        detected_beacons_t=(R @ detected_beacons_r.T).T + np.array([tx,ty])
        residuals = (detected_beacons_t - real_beacons_t).flatten()
        return residuals
    
    def estimate(self, detected_beacons_r):
        result = least_squares(
            fun = self.objective_function,
            x0=(self.lastOdom.theta, self.lastOdom.x,self.lastOdom.y),
            args=(detected_beacons_r,REAL_BEACONS_T),
            method='lm'
        )

        theta_opt,tx_opt,ty_opt=result.x
        if result.cost <  MAX_COST :
            self.lidarPos = robot_pb.Position( x = tx_opt, y = ty_opt, theta=theta_opt)
        self.pub_lidar.send(self.lidarPos)
        # print(f"Least Square : {theta_opt:.2f},{int(tx_opt)},{int(ty_opt)}\n")

if __name__ == "__main__":
    d = BeaconFinder()

    while ecal_core.ok():
        time.sleep(0.01)
