#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 28 11:44:54 2024

@author: Vivien Pravong
"""

import numpy as np
import scipy as sp



def get_inliers(lidar_angles_mpipi, lidar_distances, d_min, d_max):
    """
    Keep lidar measurements whose distance is between d_min and d_max

    Parameters
    ----------
    ldr_angles_mpipi : (N,) Array of float
                        Must be a vector containing the N angles from -pi to pi (radian) provided by the lidar measurements
    ldr_distances    : (N,) Array of float
                        Must be a vector containing the N distances (meter) provided by the lidar measurements
    d_min            : float
                        Minimum distance (meter)
    d_max            : float
                        Maximum distance (meter)

    Returns
    -------
    data_inliers : (2,M) Array of float (M < N)
                    Array containing the angles and the distances that are kept as inliers
    """
    index_inliers = np.where((d_min < lidar_distances) & (lidar_distances < d_max))[0]
    angle_inliers = lidar_angles_mpipi[index_inliers]
    dist_inliers = lidar_distances[index_inliers]

    data_inliers = np.vstack((angle_inliers, dist_inliers))

    return data_inliers


def predict_angle_dist(chi, landmarks, ldr_offset):
    """
    Compute an estimation of the polar coordinates [angle (rad); distance (meter)] of all landmarks, based on their known position, the estimated robot pose and the lidar offsets w.r.t the body

    Parameters
    ----------
    chi        : (3,3) Array of float
                 Transformation matrix in SE(2) representing the robot pose
    landmarks  : (2,Nlm) Array of float
                 Landmarks position w.r.t the world frame where Nlm is the total number of landmarks
    ldr_offset : (3,) Array of float
                 Heading (degree) and positions (meter) offsets of the lidar w.r.t the body frame

    Returns
    -------
    angle_dist_est : (2,Nlm) Array of float
                     Estimation of the polar coordinates of all landmarks w.r.t the lidar frame
    """
    offset_th = ldr_offset[0] * np.pi / 180
    offset_x = ldr_offset[1]
    offset_y = ldr_offset[2]
    offset_Mat = np.array(
        [
            [np.cos(offset_th), -np.sin(offset_th), offset_x],
            [np.sin(offset_th), np.cos(offset_th), offset_y],
            [0, 0, 1],
        ]
    )

    angle_dist_est = np.zeros((2, landmarks.shape[1]))
    for i in range(landmarks.shape[1]):
        Landmark_l = (
            np.linalg.inv(offset_Mat)
            @ np.linalg.inv(chi)
            @ np.hstack((landmarks[:, i], 1))
        )
        d = np.linalg.norm(Landmark_l[:2])
        a = np.arctan2(Landmark_l[1], Landmark_l[0])

        angle_dist_est[:, i] = [a, d]

    return angle_dist_est


def get_points_of_interest(data_inliers, angle_dist_est, landmarks, ldr_offset, chi, R):
    """
    Computes the landmarks points-of-interest (poi) w.r.t the body frame as a vector ymeas of size (2xNvlm,) where Nvlm is the number of visible landmarks.

    Parameters
    ----------
    data_inliers   : (2,M) Array of float
                      All angles and distances of interest given by the lidar
    angle_dist_est : (2,Nlm) Array of float
                      Estimation of the polar coordinates of all landmarks w.r.t the lidar frame where Nlm is the total number of landmarks (Nlm >= Nvlm)
    ldr_offset     : (3,) Array of float
                      Heading (degree) and positions (meter) offsets of the lidar w.r.t the body frame
    landmarks : (3,)
    chi            : (3,3) Array of float
                      Transformation matrix in SE(2) representing the robot pose
    R              : (2,2) Array of float
                      Covariance matrix of the measurement noises

    Returns
    -------
    ymeas            : (2xNvlm,) Array of float
                        Measured positions of the landmarks w.r.t the body frame, given as a flat array (measurement vector that will be used for Kalman correction)
    pos_poi_b        : (2,Nvlm) Array of float
                        Measured points-of-interest of the landmarks w.r.t the body frame (same data as ymeas but the positions are placed one next to the other)
    pos_poi_w        : (2,Nvlm) Array of float
                        Measured points-of-interest of the landmarks w.r.t the world frame (approximative since its based on the estimated robot pose)
    angle_dist_meas  : (2,Nvlm) Array of float
                       Measurement of the points of interest (in polar coordinates w.r.t to the lidar frame)
    index_visible_lm : (Nvlm,) Array of int
                       Indexes of the visible landmarks
    dimy             : int
                       dimy = 2xNvlm -> Dimension of the measurement vector
    sqrtR            : (2xNvlm,2xNvlm) Array of float
                       square-root of the new covariance matrix of the measurement noises
    """
    pos_poi_b = []
    pos_poi_w = []

    #print("data_inliers",data_inliers)
    offset_th = ldr_offset[0] * np.pi / 180
    offset_x = ldr_offset[1]
    offset_y = ldr_offset[2]

    angle_dist_meas = []
    ymeas = []
    diag_sqrtR = []
    index_visible_lm = []

    r_th = R[0, 0]
    r_x = R[1, 1]

    for i in range(angle_dist_est.shape[1]):
        error_angle = np.arctan2(np.sin(data_inliers[0] - angle_dist_est[0, i]),np.cos(data_inliers[0] - angle_dist_est[0, i]),)
        error_dist = data_inliers[1] - angle_dist_est[1, i]
        # print("error", error_angle)
        # print("error", error_dist)
        error = np.vstack((error_angle, error_dist))
        # print("error:",error)
        # Find pivot point
        # print(np.linalg.norm(error, axis=0))
        idx_pivot = np.argmin(np.linalg.norm(error, axis=0))

        # Find pivot nearest neighbors
        neighbors_angle = np.arctan2(
            np.sin(data_inliers[0] - data_inliers[0, idx_pivot]),
            np.cos(data_inliers[0] - data_inliers[0, idx_pivot]),
        )
        neighbors_dist = data_inliers[1] - data_inliers[1, idx_pivot]
        neighbors_cost = np.vstack((neighbors_angle, neighbors_dist))
        nns = np.where(np.linalg.norm(neighbors_cost, axis=0) <= 0.2)[0]  # 0.1 is a tuning parameter, it is up to you to tune it, in function of your intuition

        # Get candidate based on the mean of the nearest neighbors
        candidate_angle = np.mean(data_inliers[0, nns])
        candidate_dist = np.mean(data_inliers[1, nns])

        a = candidate_angle
        d = candidate_dist
        pos_poi_l = np.array([d * np.cos(a), d * np.sin(a)])
        min_error = np.abs(np.vstack((candidate_angle, candidate_dist)).squeeze()- angle_dist_est[:, i])
        tmp_b = np.array(
            [
                [np.cos(offset_th), -np.sin(offset_th), offset_x],
                [np.sin(offset_th), np.cos(offset_th), offset_y],
                [0, 0, 1],
            ]
        ) @ np.hstack((pos_poi_l, 1))
        tmp_r = chi @ np.hstack((tmp_b[:2], 1))

        landmark_error_r = tmp_b[:2] - landmarks[:,i]

        # Keep candidate if the error is "low enough", otherwise consider it as lost
        #if min_error[0] <= 30 * np.pi / 180 and min_error[1] <= 0.1:
        if np.linalg.norm(landmark_error_r) <= 10:
            angle_dist_meas.append([a, d])
            index_visible_lm.append(i)
            ymeas.append(tmp_b[:2])
            pos_poi_b.append(tmp_b[:2])
            pos_poi_w.append(tmp_r[:2])
            diag_sqrtR.append([np.sqrt(r_th), np.sqrt(r_x)])

    ymeas = np.array(ymeas).flatten()
    pos_poi_b = np.array(pos_poi_b).T.squeeze()
    pos_poi_w = np.array(pos_poi_w).T.squeeze()
    angle_dist_meas = np.array(angle_dist_meas).T.squeeze()
    index_visible_lm = np.array(index_visible_lm)
    dimy = len(ymeas)
    sqrtR = np.diag(np.array(diag_sqrtR).flatten())

    return ymeas, pos_poi_b, pos_poi_w, angle_dist_meas, index_visible_lm, dimy, sqrtR


class Kalman:
    def __init__(
        self,
        theta0=0,
        X0=np.zeros(2),
        P0=np.eye(3),
        Q=np.eye(3),
        R=np.eye(3),
        alpha=1,
        beta=0,
        kappa=0,
    ):
        self.theta = theta0
        self.X = X0
        self.P = P0
        self.S = np.linalg.cholesky(P0)
        self.dimx = np.shape(self.P)[0]
        self.Q = Q
        self.sqrtQ = np.linalg.cholesky(Q)
        self.R = R
        self.sqrtR = np.linalg.cholesky(R)
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.time = []
        Rot0 = np.array(
            [[np.cos(theta0), -np.sin(theta0)], [np.sin(theta0), np.cos(theta0)]]
        )
        X0_column = X0.reshape(-1, 1)
        chi0 = np.block([[Rot0, X0_column], [0, 0, 1]])
        self.chi = chi0

    def chi2state(self, chi):
        """
        Extract state variables [theta,X] in R^3 from variable chi in Lie group SE(2). Additionaly extract the rotation matrix Rot in SO(2).

        Parameters
        ---------
        chi : (3,3) Array of float
              Transformation matrix in SE(2) representing the robot pose

        Returns
        -------
        Rot   : (2,2) Array of float
                Rotation matrix in SO(2)
        theta : float
                Heading (radian)
        X     : (2,) Array of float
                2D position [x,y] (meter)
        """
        Rot = chi[:2, :2]
        theta = np.arctan2(Rot[1, 0], Rot[0, 0])
        X = chi[:2, 2]
        return Rot, theta, X

    def state2chi(self, theta, X):
        """
        Transforms state variables [theta,X] into its equivalent in SE(2)

        Parameters
        ----------
        theta : float
                heading (radian)
        X     : (2,) Array of float
                2D position X = [x,y] (meter)

        Returns
        -------
        chi : (3,3) Array of float
              Transformation matrix in SE(2) representing the robot pose
        """
        Rot = np.array(
            [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
        )
        X_column = X.reshape(-1, 1)
        chi = np.block([[Rot, X_column], [0, 0, 1]])
        return chi

    def compute_weights(self, n):
        """
        Computes the 2n+1 mean weights Wm, the 2n+1 covariance weigths Wc and the dispersion coefficient lambda_ of the sigma-points in the Unscented Transform

        Parameters
        ----------
        n     : float
                dimension of the state.

        Returns
        -------
        Wm      : (2n+1,) Array of float
                  Mean weights.
        Wc      : (2n+1,) Array of float
                  Covariance weights.
        lambda_ : float
                  Dispersion of the sigma-points.
        """
        lambda_ = self.alpha**2 * (n + self.kappa) - n
        Wm = np.full(2 * n + 1, 1 / (2 * (n + lambda_)))
        Wc = np.copy(Wm)
        Wm[0] = lambda_ / (n + lambda_)
        Wc[0] = Wm[0] + (1 - self.alpha**2 + self.beta)
        return Wm, Wc, lambda_

    def expSE2(self, ksi):
        """
        Exponential map of the Lie group SE(2).
        Allows to map a given vector ksi (living in the Lie algebra se(2)) into a matrix chi (living in the Lie group SE(2))

        Parameters
        ----------
        ksi : (3,) Array of float
              Vector representation in the Lie algebra se(2).
              X = [theta,ksi2,ksi3]
              (See "Unscented Kalman Filter on Lie Groups", 2017, M.Brossard, Appendix A for more details)

        Returns
        -------
        chi : (3,3) Array of float
              Transformation matrix in SE(2) representing the robot pose
        """
        th = ksi[0]
        if th == 0:
            chi = np.block([[np.eye(2), ksi[1:].reshape(-1, 1)], [np.zeros((1, 2)), 1]])
        else:
            R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
            V = (1 / th) * np.array(
                [[np.sin(th), -1 + np.cos(th)], [1 - np.cos(th), np.sin(th)]]
            )
            Vksi = np.dot(V, ksi[1:])
            chi = np.block([[R, Vksi.reshape(-1, 1)], [np.zeros((1, 2)), 1]])
        return chi

    def invSE2(self, chi):
        """
        Compute the inverse of the matrix chi of SE(2)

        Parameters
        ----------
        chi : (3,3) Array of float
              Transformation matrix in SE(2) representing the pose

        Returns
        -------
        chi_inv : (3,3) Array of float
                   inverse of self.chi
        """
        R = chi[:2, :2]
        t = chi[:2, 2]
        v = -R.T @ t
        chi_inv = np.block([[R.T, v.reshape(-1, 1)], [np.zeros((1, 2)), 1]])
        return chi_inv

    def logSE2(self, chi):
        """
        Logarithm map of the Lie group SE(2).
        Allows to map a given matrix chi in the Lie group SE(2) into a vector in the Lie algebra se(2)

        Parameters
        ----------
        chi : (3,3) Array of float
              Transformation matrix in SE(2) representing the pose

        Returns
        -------
        ksi : (3,) Array of float
              Vector representation in the Lie algebra se(2).
              X = [theta,ksi2,ksi3]
              (See "Unscented Kalman Filter on Lie Groups", 2017, M.Brossard, Appendix A for more details)
        """
        Rot = chi[:2, :2]
        theta = np.arctan2(Rot[1, 0], Rot[0, 0])
        X = chi[:2, 2]

        if theta == 0:
            ksi = np.concatenate(([theta], X))
        else:
            A = np.sin(theta) / theta
            B = (1 - np.cos(theta)) / theta
            V_inv = 1 / (A**2 + B**2) * np.array([[A, B], [-B, A]])

            ksi = np.concatenate(([theta], V_inv.dot(X)))
        return ksi

    def h_fullvec(self, chi, r):
        #theta = np.arctan2(chi[1,0], chi[0,0]) + r[0]
        tmp_y = chi @ np.array([0,0,1]) + np.array([r[0],r[1],0])
        y = tmp_y[:2]
        Y = y#np.array([theta, *y])
        return Y

    def h_lm_pos(self, chi, landmarks, index_visible_lm, r):
        """
        Output/Observation model. Computes an estimation of the position of all visible landmarks in the body frame, knowing the estimated body pose chi

        Parameters
        ----------
        chi              : (3,3) Array of float
                            Transformation matrix in SE(2) representing the robot pose
        landmarks        : (2,Nlm) Array of float
                            Array containing the absolute position of all landmarks where Nlm is the total number of landmarks
        index_visible_lm : (Nvlm,) Array of int
                            Array containing the indexes of the visible landmarks where Nvlm is the number of visible landmarks (Nvlm <= Nlm)
        r                : (2xNvlm,) Array of float
                            Noise vector

        Returns
        -------
        Y : (2xNvlm,) Array of float
            Estimated output vector. Contains the estimated of the 2D position of all visible landmarks w.r.t the body frame
        """
        Y = []
        for i in index_visible_lm:
            Yi = np.linalg.inv(chi) @ np.hstack((landmarks[:, i], 1))
            Y.append(Yi[:2])
        Y = np.array(Y).flatten() + r
        return Y

    def pred(self, dt, u):
        """
        Perform the prediction step of the Kalman Filter / Compute a first estimate of the state via the kinematical model.
        It changes the values of self.chi, self.S and self.P according to a certain input u performed during the time step dt.
        (See "Unscented Kalman Filter on Lie Groups", 2017, M.Brossard, algorithm 2 for more details)

        Parameters
        ----------
        dt : float
             Time step (sec) between two consecutive change of pose
        u  : (3,) Array of float
             Model input given as an array of 3 inputs.
             The first one corresponds to the angular velocity (rad/sec) of the body,
             the 2nd and 3rd inputs correspond to a linear velocity along the x and y coordinates (meter/sec) w.r.t the body frame

        Returns
        -------
        self.chi : (3,3) Array of float
                   Predicted state in SE(2)
        self.S   : (3,3) Array of float
                   Cholesky decomposition (~basically the square-root) of the covariance matrix P
        self.P   : (3,3) Array of float
                   Covariance matrix of the predicted state

        """
        # Augment the state with the process noise
        S_aug = np.block(
            [
                [self.S, np.zeros((self.S.shape[0], self.sqrtQ.shape[1]))],
                [np.zeros((self.sqrtQ.shape[0], self.S.shape[1])), self.sqrtQ],
            ]
        )
        n_aug = S_aug.shape[0]
        x_aug = np.zeros(n_aug)

        # Compute the weights
        Wm, Wc, lambda_ = self.compute_weights(n_aug)

        # Generate sigma-points based on the covariance
        eta = np.sqrt(n_aug + lambda_)
        SigPts = eta * np.hstack((x_aug.reshape([n_aug, 1]), -S_aug.T, S_aug.T))

        chiAnt = self.chi  # Keep in mind anterior pose before propagation
        self.chi = self.chi @ self.expSE2(u * dt)  # Perform the propagation of the mean from t-1 to t
        chi_inv = self.invSE2(self.chi)  # Compute the inverse of chi

        # Propagate each sigma-point through the nonlinear function
        for j in range(1, 2 * n_aug + 1):
            ksi_j = SigPts[: self.dimx, j]
            qj = SigPts[self.dimx :, j]
            uj = np.array([u[0], u[1], u[2]])
            chi_j = chiAnt @ self.expSE2(ksi_j)
            chi_j = chi_j @ self.expSE2((uj + qj) * dt)
            Xi_j = chi_inv @ chi_j
            SigPts[: self.dimx, j] = self.logSE2(Xi_j)

        # Compute the covariance of the prediction
        WSigPts = np.sqrt(Wm[1:]) * SigPts[: self.dimx, 1:]
        _, RSx = np.linalg.qr(WSigPts.T)
        self.S = RSx[: self.dimx, : self.dimx]
        self.P = self.S.T @ self.S

        return self.chi, self.S, self.P

    def corr(self, ymeas, landmarks, index_visible_lm, dimy, sqrtR):
        """
        Perform the correction step of the Kalman Filter / Compute the correction of the estimate using a measurement ymeas.
        (See "Unscented Kalman Filter on Lie Groups", 2017, M.Brossard, algorithm 1 for more details)

        Parameters
        ----------
        ymeas            : (dimy,) Array of float
                           Measurement vector. It must contain the position of each visible landmarks w.r.t the body frame
        landmarks        : (2,Nlm) Array of float
                           Landmarks position w.r.t the world frame where Nlm is the total number of landmarks
        index_visible_lm : (Nvlm,) Array of int
                           Indexes of the visible landmarks
        dimy             : int
                           dimy = 2xNvlm -> Dimension of the measurement vector
        sqrtR            : (dimy,dimy) Array of float
                           Square-root of the covariance matrix of the measurement

        Returns
        -------
        self.chi : (3,3) Array of float
                   Corrected state in SE(2)
        self.S   : (3,3) Array of float
                   Cholesky decomposition (~basically the square-root) of the covariance matrix P
        self.P   : (3,3) Array of float
                   Covariance matrix of the corrected state
        """
        # Augment the state with the measurement noise
        S_aug = np.block(
            [
                [self.S, np.zeros((self.S.shape[0], sqrtR.shape[1]))],
                [np.zeros((sqrtR.shape[0], self.S.shape[1])), sqrtR],
            ]
        )
        n_aug = S_aug.shape[0]
        x_aug = np.zeros(n_aug)

        # Compute the weights
        Wm, Wc, lambda_ = self.compute_weights(n_aug)

        # Generate sigma-points based on the covariance
        eta = np.sqrt(n_aug + lambda_)
        SigPts = eta * np.hstack((x_aug.reshape([n_aug, 1]), -S_aug.T, S_aug.T))

        # Propagate each sigma-point through the nonlinear function
        Y = np.zeros((dimy, 2 * n_aug + 1))
        for j in range(2 * n_aug + 1):
            ksi_j = SigPts[:self.dimx, j]
            rj = SigPts[self.dimx:, j]
            chi_j = self.chi @ self.expSE2(ksi_j)
            Y[:, j] = self.h_lm_pos(chi_j, landmarks, index_visible_lm, rj)

        # Compute the prediction of the measurement
        ypred = np.sum(Wm * Y, axis=1)
        # Compute the covariance of the prediction of the measurement
        WY = np.sqrt(Wc[1:]) * (Y[:, 1:] - ypred[:, np.newaxis])
        _, RSy = np.linalg.qr(WY.T)
        Sy = RSy[:dimy, :dimy]
        Uy = np.sqrt(abs(Wc[0])) * (Y[:, 0] - ypred)
        Sy = sp.linalg.cholesky(Sy.T @ Sy - np.outer(Uy, Uy))
        Py = Sy.T @ Sy

        # Compute the cross-covariance
        Pxy = np.zeros((self.dimx, dimy))
        for j in range(1, 2 * n_aug + 1):
            Pxy += Wc[j] * np.outer(SigPts[: self.dimx, j], Y[:, j] - ypred)

        # Compute the Kalman gain
        K = np.dot(Pxy, np.linalg.inv(Py))
        # Compute the innovation
        ksi_bar = K @ (ymeas - ypred)

        # Perform the correction of the estimation
        self.chi = self.chi @ self.expSE2(ksi_bar[: self.dimx])

        # Perform the correction of the covariance of the estimate
        U = K @ Sy.T
        for j in range(dimy):
            self.S = sp.linalg.cholesky(self.S.T @ self.S - np.outer(U[:, j], U[:, j]))

        self.P = self.S.T @ self.S

        return self.chi, self.S, self.P
    
    def corr_h_fullvec(self, ymeas, dimy, sqrtR):
        S_aug = np.block(
            [
                [self.S, np.zeros((self.S.shape[0], sqrtR.shape[1]))],
                [np.zeros((sqrtR.shape[0], self.S.shape[1])), sqrtR],
            ]
        )
        n_aug = S_aug.shape[0]
        x_aug = np.zeros(n_aug)
        Wm, Wc, lambda_ = self.compute_weights(n_aug)
        eta = np.sqrt(n_aug + lambda_)
        SigPts = eta * np.hstack((x_aug.reshape([n_aug, 1]), -S_aug.T, S_aug.T))
        Y = np.zeros((dimy, 2 * n_aug + 1))
        for j in range(2 * n_aug + 1):
            ksi_j = SigPts[:self.dimx, j]
            rj = SigPts[self.dimx:, j]
            chi_j = self.chi @ self.expSE2(ksi_j)
            Y[:, j] = self.h_fullvec(chi_j, rj)
        ypred = np.sum(Wm * Y, axis=1)
        WY = np.sqrt(Wc[1:]) * (Y[:, 1:] - ypred[:, np.newaxis])
        _, RSy = np.linalg.qr(WY.T)
        Sy = RSy[:dimy, :dimy]
        Uy = np.sqrt(abs(Wc[0])) * (Y[:, 0] - ypred)
        Sy = sp.linalg.cholesky(Sy.T @ Sy - np.outer(Uy, Uy))
        Py = Sy.T @ Sy
        Pxy = np.zeros((self.dimx, dimy))
        for j in range(1, 2 * n_aug + 1):
            Pxy += Wc[j] * np.outer(SigPts[: self.dimx, j], Y[:, j] - ypred)
        K = np.dot(Pxy, np.linalg.inv(Py))
        ksi_bar = K @ (ymeas - ypred)
        self.chi = self.chi @ self.expSE2(ksi_bar[: self.dimx])
        U = K @ Sy.T
        for j in range(dimy):
            self.S = sp.linalg.cholesky(self.S.T @ self.S - np.outer(U[:, j], U[:, j]))
        self.P = self.S.T @ self.S
        return self.chi, self.S, self.P


# %% MAIN
if __name__ == "__main__":
    # Comment utiliser le filtre de Kalman ?

    # 1) Initialiser le filtre
    # 1.1) Donner une estimée initiale de l'état du robot [theta0,x0,y0]
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

    filter = Kalman(
        theta0=theta0, X0=X0, P0=P0, Q=Q, R=R, alpha=alpha, beta=beta, kappa=kappa
    )
    # Voilà, le filtre est initialisée, on peut désormais l'utiliser :D

    # 2) En boucle, réaliser les prédictions et corrections au fur et à mesure de l'évolution du robot et de l'acquisition des mesures
    # 2.1) La prédiction est faites à la fréquence d'acquisition des vitesses angulaires et linéaires
    # Admettons qu'on se situe à un instant quelconque et qu'on connaît l'instant précédent dans lequel on avait réalisé une précédente prédiction
    # on a alors besoin du pas de temps dt entre ces deux instants
    dt = 0.01  # sec
    u = np.array(
        [1 * np.pi / 180, 0.1, 0.1]
    )  # vecteur d'entrée du modèle. Les composantes sont la vitesse angulaire (rad/s) du gyro et les vitesses longitudinale et transversale (m/s) respectivement
    chi, S, P = filter.pred(dt, u)  # réalise la prédiction

    # 2.2) Avant de passer à la correction, on a besoin d'une mesure ymeas à un instant donné
    # Cette mesure sera fournit par le lidar mais il faut pouvoir traiter les données lidar et les transformer vers le jeu de coordonnées qui nous arrange.
    # Ici, on récupère les données 'angles' et 'distances' du lidar (sous quelconque manière que ce soit) mais il faut ABSOLUMENT les mettre sous forme de Array de taille (N,) pour pouvoir appliquer la prochaine fonction
    # Admettons qu'on ait fait ça, dans ce cas on aurait un truc du style...
    ldr_angles_mpipi = np.array(
        [-np.pi, -3 * np.pi / 4, 0, 3 * np.pi / 4, np.pi]
    )  # il faut absolument avoir préalablement borné ce vecteur de -pi à pi pour faire marcher les prochaines fonctions
    ldr_distances = np.array([4, 1, 1, 1, 4])  # les distances doivent être en mètre
    # Admettons aussi qu'on veuille exclure toutes les données inférieures à une distance de 0.1m et supérieures à 3.6m...
    d_min = 0.1
    d_max = 3.6
    data_inliers = get_inliers(
        ldr_angles_mpipi, ldr_distances, d_min, d_max
    )  # cette fonction exclue donc les données trop loins et trop proches

    # 2.3) La procédure predict_angle_dist() permet d'estimer ce que donnerait le lidar pour chaque balise de l'environnement. On a besoin de ça pour pouvoir extraire les points d'intérêt en coordonnées polaires
    # (en gros, pour pouvoir être capable de deviner quelles sont les ensembles (angle/distance) correspondant à nos balises.
    # Admettons que nos balises sont situées aux positions suivantes...
    balise1 = [-1, -1]
    balise2 = [1, 0]
    balise3 = [-1, 1]
    landmarks = np.transpose(
        np.array([balise1, balise2, balise3])
    )  # position absolue des balises (mètre)
    # Admettons aussi qu'on à des offsets du lidar par rapport au corps du robot
    offset_angulaire = 115  # en degré
    offset_pos_x = 0.06  # mètre
    offset_pos_y = 0.06
    ldr_offset = np.array([offset_angulaire, offset_pos_x, offset_pos_y])
    # Connaissant la position absolue des balises, l'offset du lidar, et une estimée (via la prédiction du Kalman) de la pose du robot,...
    # ...on peut estimer les coordonnées polaires (angle,distance) de chaque balise dans le repère LIDAR
    angle_dist_est = predict_angle_dist(chi, landmarks, ldr_offset)
    # dans 'angle_dist_est', on a donc un Array (2, Nb de balises) où la 1ère ligne correspond aux angles et la 2e correspond aux distances

    # 2.4) On peut maintenant extraire les "vraies" coordonnées polaires depuis les mesures lidar.
    # Grossomodo, à l'aide de l'estimée des coordonnées polaires faite précédemment, on recherche un point pivot (tjrs en coordonnées polaires).
    # Ce point pivot cherchera ses plus proches voisins (avec un critère à choisir selon nos intuitions)
    # Cette procédure est faite avec la fonction suivante:
    ymeas, pos_poi_b, pos_poi_w, angle_dist_meas, index_visible_lm, dimy, sqrtR = (
        get_points_of_interest(data_inliers, angle_dist_est, landmarks, ldr_offset, chi, R)
    )
    # La plupart des choses retournées par la fonction est inutile pour le Kalman mais utile pour de l'affichage (donc je laisse au cas où).
    # Les plus importants pour le Kalman sont 'ymeas','index_visible_lm','dimy','sqrtR'
    # 'index_visible_lm' indique l'indice des balises qu'on a choisie de garder (mais en soi le rejet des balises est discutable, on devrait p-ê voir ça ensemble)

    # 3) On peut finalement réaliser l'étape de correction (sauf si ymeas est vide, auquel cas on ne peut pas faire de correction puisqu'on a pas de mesure disponible)
    chi, S, P = filter.corr(ymeas, landmarks, index_visible_lm, dimy, sqrtR)

    # FIN DE LA BOUCLE
