import kalman_class
import ld06_driver as lidar
import sys

sys.path.append("/home/robot/dev/robot_rpi_2024/generated")

import lidar_data_pb2 as lidar_data

import numpy as np

from time import time, sleep


import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber

import robot_state_pb2 as hgpb

ecal_core.initialize(sys.argv, "robotStateHolder")



theta0 = 0  # doit être en radian
x0 = 0  # en mètre
y0 = 0  # en mètre
X0 = np.array([x0, y0])  # Array (2,) contenant les deux positions initiales x et y
# 1.2) Donner une estimée de notre incertitude sur l'état initiale (caractérisée par une matrice de covariance)
p0theta = (
    10 * np.pi / 180
) ** 2  # variance sur le cap initiale (ici on a par exemple ~10° d'incertitude sur le cap initiale)
p0x = (
    0.05
) ** 2  # variance sur la position x initiale (ici on a par exemple 5cm d'incertitude)
p0y = (0.05) ** 2
P0 = np.array([[p0theta, 0, 0], [0, p0x, 0], [0, 0, p0y]])
# 1.3) Régler les paramètres de bruit Q et R (caractérisant notre incertitude sur le modèle d'évolution et sur le modèle d'observation resp.)
q_theta = (1 * np.pi / 180) ** 2  # variance sur la vitesse angulaire
q_x = (0.15) ** 2  # variance sur la vitesse longitudinale
q_y = (0.15) ** 2  # variance sur la vitesse transversale
Q = np.array([[q_theta, 0, 0], [0, q_x, 0], [0, 0, q_y]])
r_1 = (
    0.001
) ** 2  # variance sur la coordonnée x de la position d'une balise dans le repère corps du robot
r_2 = (
    0.001
) ** 2  # variance sur la coordonnée y de la position d'une balise dans le repère corps du robot
R = np.array(
    [[r_1, 0], [0, r_2]]
)  # attention ! R est est de taille 2x2 ici mais cela va être amené à changer selon le nombre de balise qu'on décide de rejeter ou pas
# 1.4) Régler les paramètres de l'Unscented transformation (faire des recherche sur l'UKF pour + de détails)
alpha = 1
beta = 0
kappa = 0

#u = np.array(
#    [0, 0, 0]
#    #[1 * np.pi / 180, 0.1, 0.1]
#) # vecteur d'entrée u = [vitesse angulaire, vitesse longitudinale, vitesse transversale]
# à actualiser quand le robot sera en mouvement

filter = kalman_class.Kalman(
    theta0=theta0, X0=X0, P0=P0, Q=Q, R=R, alpha=alpha, beta=beta, kappa=kappa
)


def prediction(dt, u):
    """
    à une fréquence plus élevée que les données du lidar et du kalman. 
    Prend en compte les messages odométrie et gyro
    dt en secondes, u en rad/:s et m/s
    """
    global start, end, filter, chi, S, P

    #u = np.array([1 * np.pi / 180, 0.1, 0.1])
    # vecteur d'entrée du modèle. Les composantes sont la vitesse angulaire (rad/s) du gyro et les vitesses longitudinale et transversale (m/s) respectivement

    chi, S, P = filter.pred(dt, u) 


def callback_speed(topic_name, msg:hgpb.Speed, timestamp):
    print(msg)
    u[1], u[2] = msg.vx, msg.vy


def callback_gyro(topic_name, msg:hgpb.Ins, timestamp):
    print(msg)
    u[2] =  msg.vtheta

    

# cf. kalman_class.py pour les détails de la boucle de calcul
def callback_lidar(topic_name, msg:lidar_data.Lidar, timestamp):
    #angles: list[float], distances: list[float], _: list[float]
    
    global u, filter, chi, S, P

    print("angles : ", msg.angles)
    print("distances : ", msg.distances)
    ldr_angles = np.array(msg.angles)
    ldr_distances = np.array(msg.distances)
    
    ##TODO filtrer points avec la qualité

    
    d_min = 0.1
    d_max = 3.6

    data_inliers = kalman_class.get_inliers(
        ldr_angles, ldr_distances, d_min, d_max
    )

    print("firstdata", data_inliers)
    # coordonnées des balises dans le repère global utilisé (côté bleu)
    balise1 = np.array([-0.01 , 1])
    balise2 = np.array([3.09, 1.95])
    balise3 = np.array([3.09, 0.05])

    landmarks = np.transpose(
        np.array([balise1, balise2, balise3])
    )

    print("landmarks : " , landmarks)
    offset_angulaire = 0
    offset_x = 0
    offset_y = 0

    ldr_offset = np.array([offset_angulaire, offset_x, offset_y])

    angle_dist_est = kalman_class.predict_angle_dist(chi, landmarks, ldr_offset)

    ymeas, pos_poi_b, pos_poi_w, angle_dist_meas, index_visible_lm, dimy, sqrtR = (
        kalman_class.get_points_of_interest(data_inliers, angle_dist_est, ldr_offset, chi, R)
    )


    ####################"Data inliers emptyyy"
    chi, S, P = filter.corr(ymeas, landmarks, index_visible_lm, dimy, sqrtR)

    print("chi", chi)
    print("pos_poi_b", pos_poi_b)
    print("pos_poi_w", pos_poi_w)
    print("angle_dist_meas", angle_dist_meas)
    print("index_visible_lm", index_visible_lm)
    print("dimy", dimy)
    print("sqrtR", sqrtR)

if __name__ == '__main__':
    start = time()
    lidar_data_sub = ProtoSubscriber("lidar_data",lidar_data.Lidar)
    
    speed_sub = ProtoSubscriber("odom_speed", hgpb.Speed)

    gyro_sub = ProtoSubscriber("ins", hgpb.Ins)

    speed_sub.set_callback(callback_speed)
    gyro_sub.set_callback(callback_gyro)
   

    while(1):
        dt = 0.1
        u = np.zeros(3)
        prediction(dt,u)
        print("pos :", filter.X[0],filter.X[1],filter.theta)
        lidar_data_sub.set_callback(callback_lidar)
        sleep(dt)


    #driver = lidar.Driver(callback_lidar)
    #driver.scan()
