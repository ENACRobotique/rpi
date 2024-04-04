import kalman_class
import ld06_driver as lidar

import numpy as np

from time import time

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

u = np.array(
    [0, 0, 0]
    #[1 * np.pi / 180, 0.1, 0.1]
) # vecteur d'entrée u = [vitesse angulaire, vitesse longitudinale, vitesse transversale]
# à actualiser quand le robot sera en mouvement

filter = kalman_class.Kalman(
    theta0=theta0, X0=X0, P0=P0, Q=Q, R=R, alpha=alpha, beta=beta, kappa=kappa
)

# cf. kalman_class.py pour les détails de la boucle de calcul
def callback_lidar(angles: list[float], distances: list[float], _: list[float]):
    global start, end, u, filter

    end = time()
    dt = end - start
    chi, S, P = filter.pred(u, dt)
    start = time()

    ldr_angles = np.array(angles)
    ldr_distances = np.array(distances)
    
    d_min = 0.1
    d_max = 3.6

    data_inliers = kalman_class.get_inliers(
        ldr_angles, ldr_distances, d_min, d_max
    )

    # coordonnées des balises dans le repère global utilisé (côté bleu)
    balise1 = np.array([-0.01 , 1])
    balise2 = np.array([3.09, 1,95])
    balise3 = np.array([3.09, 0.05])

    landmarks = np.transpose(
        np.array([balise1, balise2, balise3])
    )

    offset_angulaire = 0
    offset_x = 0
    offset_y = 0

    ldr_offset = np.array([offset_angulaire, offset_x, offset_y])

    angle_dist_est = kalman_class.predict_angle_dist(chi, landmarks, ldr_offset)

    ymeas, pos_poi_b, pos_poi_w, angle_dist_meas, index_visible_lm, dimy, sqrtR = (
        kalman_class.get_points_of_interest(data_inliers, angle_dist_est, ldr_offset, chi, R)
    )

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
    driver = lidar.Driver(callback_lidar)
    driver.scan()
