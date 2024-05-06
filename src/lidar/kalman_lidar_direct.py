import kalman_class
import ld06_driver as lidar
import sys



import generated.lidar_data_pb2 as lidar_data
import generated.lidar_data_pb2 as lidar_data

import numpy as np

from time import time, sleep


import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber

import generated.robot_state_pb2 as hgpb


global filter

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

def callback_odom_pos(topic_name, msg:hgpb.Position, timestamp):
    #print(msg)
    global theta_odom, X_odom
    theta_odom, x, y = msg.theta, msg.x*10**(-3), msg.y*10**(-3)
    X_odom = np.array([x,y])

def callback_speed(topic_name, msg:hgpb.Speed, timestamp):
    #print(msg)
    u[1], u[2] = msg.vx*10**(-2), msg.vy*10**(-2)


def callback_gyro(topic_name, msg:hgpb.Ins, timestamp):
    #print(msg)
    u[0] =  msg.vtheta

    

# cf. kalman_class.py pour les détails de la boucle de calcul
def callback_lidar(topic_name, msg:lidar_data.Lidar, timestamp):
    #angles: list[float], distances: list[float], _: list[float]
    
    global u, filter, chi, S, P, ymeas, landmarks, index_visible_lm, dimy, sqrtR

    #garder mesures de bonne qualité avant tout traitement
    index_to_keep = np.where(np.array(msg.quality) > 160)[0]
    lidar_angles_to_keep = np.array(msg.angles)[index_to_keep]
    lidar_distances_to_keep = np.array(msg.distances)[index_to_keep]

    lidar_angles = lidar_angles_to_keep*np.pi/180
    lidar_angles_mpipi = np.arctan2(np.sin(lidar_angles),np.cos(lidar_angles))
    lidar_distances = lidar_distances_to_keep*10**(-3)
    
    d_min = 0.1
    d_max = 3.6

    offset_angulaire = 0
    offset_x = 0
    offset_y = 0
    ldr_offset = np.array([offset_angulaire, offset_x, offset_y])

    data_inliers = kalman_class.get_inliers(lidar_angles_mpipi, lidar_distances, d_min, d_max)
    angle_dist_est = kalman_class.predict_angle_dist(chi, landmarks, ldr_offset)
    ymeas, pos_poi_b, pos_poi_w, angle_dist_meas, index_visible_lm, dimy, sqrtR = kalman_class.get_points_of_interest(data_inliers, angle_dist_est, ldr_offset, chi, R)

    n = len(pos_poi_w)
    #print(pos_poi_b)
    if n >= 1:    
        b1.send(hgpb.Position(x = (pos_poi_w[0][0])*1000,y =  (pos_poi_w[0][1])*1000 , theta = 0))
        if n >=2:
            b2.send(hgpb.Position(x = (pos_poi_w[1][0])*1000,y =  (pos_poi_w[1][1])*1000, theta = 0))
            if n>=3:
                b3.send(hgpb.Position(x = (pos_poi_w[2][0])*1000,y =  (pos_poi_w[2][1])*1000, theta = 0))

    chi, S, P = filter.corr(ymeas, landmarks, index_visible_lm, dimy, sqrtR)

#def callback_lidar_pos(topic_name, msg:hgpb.Position, timestamp):
#    global chi, S, P
#    #ymeas = np.array([msg.theta,msg.x*10**(-3),msg.y*10**(-3)])
#    ymeas = np.array([msg.x*10**(-3),msg.y*10**(-3)])
#    chi, S, P = filter.corr_h_fullvec(ymeas, dimy, sqrtR)
    

    # print("chi", chi)
    #print("pos_poi_b", pos_poi_b)
    # print("pos_poi_w", pos_poi_w)
    # print("angle_dist_meas", angle_dist_meas)
    # print("index_visible_lm", index_visible_lm)
    # print("dimy", dimy)
    # print("sqrtR", sqrtR)

if __name__ == '__main__':

    ecal_core.initialize(sys.argv, "lidar_kalman")
    b1 = ProtoPublisher("b1",hgpb.Position)
    b2 = ProtoPublisher("b2",hgpb.Position)
    b3 = ProtoPublisher("b3",hgpb.Position)
    start = time()
    lidar_data_sub = ProtoSubscriber("lidar_data",lidar_data.Lidar)
    lidar_pos_pub = ProtoPublisher("kalman_pos",hgpb.Position)
    lidar_pos = hgpb.Position()

    odom_pos_sub = ProtoSubscriber("odom_pos", hgpb.Position)
    speed_sub = ProtoSubscriber("odom_speed", hgpb.Speed)
    gyro_sub = ProtoSubscriber("ins", hgpb.Ins)
    lidar_pos_sub = ProtoSubscriber("lidar_pos", hgpb.Position)

    theta0 = np.pi/180 * 45  # doit être en radian
    x0 = 1.5  # en mètre
    y0 =  1.# en mètre
    X0 = np.array([x0, y0])  # Array (2,) contenant les deux positions initiales x et y
    # 1.2) Donner une estimée de notre incertitude sur l'état initiale (caractérisée par une matrice de covariance)
    p0theta = (60 * np.pi / 180) ** 2  # variance sur le cap initiale (ici on a par exemple ~10° d'incertitude sur le cap initiale)
    p0x = (0.5) ** 2  # variance sur la position x initiale (ici on a par exemple 5cm d'incertitude)
    p0y = (0.5) ** 2
    P0 = np.array([[p0theta, 0, 0], [0, p0x, 0], [0, 0, p0y]])
    # 1.3) Régler les paramètres de bruit Q et R (caractérisant notre incertitude sur le modèle d'évolution et sur le modèle d'observation resp.)
    q_theta = (1 * np.pi / 180) ** 2  # variance sur la vitesse angulaire
    q_x = (0.1) ** 2  # variance sur la vitesse longitudinale
    q_y = (0.1) ** 2  # variance sur la vitesse transversale
    Q = np.array([[q_theta, 0, 0], [0, q_x, 0], [0, 0, q_y]])
    r_1 = (0.05) ** 2  # variance sur la coordonnée x de la position d'une balise dans le repère corps du robot
    r_2 = (0.05) ** 2  # variance sur la coordonnée y de la position d'une balise dans le repère corps du robot
    R = np.array([[r_1, 0], [0, r_2]])  # attention ! R est est de taille 2x2 ici mais cela va être amené à changer selon le nombre de balise qu'on décide de rejeter ou pas
    sqrtR = np.array([[np.sqrt(r_1), 0], [0, np.sqrt(r_2)]])
    dimy = 2
    # 1.4) Régler les paramètres de l'Unscented transformation (faire des recherche sur l'UKF pour + de détails)
    alpha = 1
    beta = 0
    kappa = 0

    #u = np.array(
    #    [0, 0, 0]
    #    #[1 * np.pi / 180, 0.1, 0.1]
    #) # vecteur d'entrée u = [vitesse angulaire, vitesse longitudinale, vitesse transversale]
    # à actualiser quand le robot sera en mouvement

    filter = kalman_class.Kalman(theta0=theta0, X0=X0, P0=P0, Q=Q, R=R, alpha=alpha, beta=beta, kappa=kappa)
    chi = filter.state2chi(theta0,X0)
    state = filter.chi2state(chi)

    u = np.zeros(3)
    ymeas = np.array([])
    # coordonnées des balises dans le repère global utilisé (côté bleu)
    balise1 = np.array([3.09 , 1])
    balise2 = np.array([-0.09, 0.05])
    balise3 = np.array([-0.09, 1.95])
    balise4 = np.array([1.36, 2.14])
    landmarks = np.transpose(np.array([balise1, balise2, balise3, balise4]))
    index_visible_lm = np.array([])

    odom_pos_sub.set_callback(callback_odom_pos)
    speed_sub.set_callback(callback_speed)
    gyro_sub.set_callback(callback_gyro)
    lidar_data_sub.set_callback(callback_lidar)
    #lidar_pos_sub.set_callback(callback_lidar_pos)

    sleep(1)
   
    while(1):
        dt = 0.1
        #u = np.zeros(3)    
        prediction(dt,u)
        #chi = filter.state2chi(theta_odom,X_odom)
        state = filter.chi2state(chi)
        lidar_pos.x = state[2][0]*1000
        lidar_pos.y = state[2][1]*1000
        lidar_pos.theta = state[1]
        print("pos :",int(lidar_pos.x),int(lidar_pos.y),int(lidar_pos.theta*180/np.pi))
        lidar_pos_pub.send(lidar_pos)
        sleep(dt)


    #driver = lidar.Driver(callback_lidar)
    #driver.scan()
