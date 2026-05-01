#!/usr/bin/env python3
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.proto.core import Publisher as ProtoPublisher
from ecal.msg.common.core import ReceiveCallbackData
import common_pb2 as cpb2
import lidar_data_pb2 as lpb2
from robot_state_pb2 import Arucos, Side
import numpy as np
from numpy.linalg import inv
import time
import socket
from functools import partial
from loca_lidar import BEACONS_BLUE, BEACONS_YELLOW
from common import Pos
from typing import Callable
from math import atan2

# Question, sachant que : 
# - le retour d'info des encodeurs est très rapide (1kHz)
# - Les encodeurs me donnent la même chose que ma commande [V, omega], avec le retard et les imperfections de l'asservissement entre les deux.
# Est-ce que je dois plutôt faire :
# 1- prédiction avec mes commandes, update avec lidar + gyro + encodeurs
# 2- prédiction avec les encodeurs, update avec lidar + gyro (les encodeurs deviennent mes commandes)
# --> je fais 2- pour l'instant.

TELEPOT_SERVER = ("localhost", 47269)
"""

State variable      X  = [x, y, theta, V, omega]
Command vector      U  = [V, omega]
Lidar measures      Yl = [x, y, theta]
gyro measures       Yg = [omega]
encoders measures   Ye = [V, omega]

Hypothesis (low confidence): acceleration and angular acceleration are nulls.

f(X, U) = [V*cos(theta), V*sin(theta), omega, 0, 0]


https://nbviewer.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/11-Extended-Kalman-Filters.ipynb


"""

Hl = np.array([[1,0,0,0,0], [0,1,0,0,0], [0,0,1,0,0]])
Hg = np.array([[0,0,0,0,1]])
He = np.array([[0,0,0,1,0], [0,0,0,0,1]])
Hc = np.array([[1,0,0,0,0], [0,1,0,0,0], [0,0,1,0,0]])


class EkfDiff:
    def __init__(self, X0, dt, lidar_var, gyro_var, enc_var, cam_var , beacons_var, model_var) -> None:
        pass
        if not ecal_core.is_initialized():
            ecal_core.initialize("RadarQt receiver")
        self.lidar_sub = ProtoSubscriber(cpb2.Position, "lidar_pos")
        self.lidar_sub.set_receive_callback(self.handle_lidar_data)
        self.gyro_sub = ProtoSubscriber(cpb2.Ins, "ins")
        self.gyro_sub.set_receive_callback(self.handle_gyro)
        self.encoders_sub = ProtoSubscriber(cpb2.Speed, "odom_speed")
        self.encoders_sub.set_receive_callback(self.handle_encoders)
        self.team_sub = ProtoSubscriber(Side, "color")

        self.team = None
        self.team_sub.set_receive_callback(self.handle_team)
        self.camera_sub = ProtoSubscriber(Arucos, "Arucos_world")
        self.camera_sub.set_receive_callback(self.handle_camera)
        self.reset_sub = ProtoSubscriber(cpb2.Position, "reset")
        self.reset_sub.set_receive_callback(self.reset_pos)

        # self.encoders_sub = ProtoSubscriber(lpb2.Balises, "balises_near_odom")
        # self.encoders_sub.set_receive_callback(self.handle_beacons)

        #self.encoders_sub.set_receive_callback(self.handle_commands)
        # self.commands_sub = ProtoSubscriber(cpb2.Speed, "speed_cmd")
        # self.commands_sub.set_receive_callback(self.handle_encoders)
        # self.cmd_sub = ProtoSubscriber(cpb2.Speed, "speed_cons")
        # self.cmd_sub.set_receive_callback(self.handle_speed_cons)
        self.ekf_pub = ProtoPublisher(cpb2.Position, "ekf_cam_pos")


        self.BEACONS = BEACONS_BLUE

        # to plot data
        self.so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.X = X0
        self.dt = dt

        self.P = np.eye(5)



        ## observation noises
        ## variance du bruit de mesure lidar
        lv_x, lv_y, lv_theta = lidar_var
        self.Rl = np.array([[lv_x, 0, 0], [0, lv_y, 0], [0, 0, lv_theta]])

        self.Rg = np.array([[np.radians(gyro_var)]])

        enc_var_v, enc_var_omega = enc_var
        self.Re = np.array([[enc_var_v, 0], [0, enc_var_omega]])

        cv_x, cv_y, cv_theta = cam_var
        self.Rc = np.array([[cv_x, 0, 0], [0, cv_y, 0], [0, 0, cv_theta]])

        beacons_var_r, beacons_var_alpha = beacons_var
        self.Rb = np.array([[beacons_var_r, 0], [0, beacons_var_alpha]])


        ## system noise
        self.Q = np.array([
            [.25*dt**4, .5*dt**3,        0,         0,        0],
            [ .5*dt**3,    dt**2,        0,         0,        0],
            [        0,        0,        dt,        0,        0],
            [        0,        0,        0,        dt,        0],
            [        0,        0,        0,         0,       dt]
        ]) @ np.diag([*model_var])
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        self.lidar_sub.remove_receive_callback()
        self.gyro_sub.remove_receive_callback()
        self.encoders_sub.remove_receive_callback()
        # self.cmd_sub.remove_receive_callback()
        pass
    
    @staticmethod
    def f(X, _U):
        """
        Model. Returns what should be the derivative of X, given X and U (command vector).
        f(X, U) = [V*cos(theta), V*sin(theta), omega, 0, 0]
        """
        # state vector
        _x, _y, theta, v, omega = X
        # command vector
        #v_u, omega_u = _U
        # using only the state estimation, not the commands
        Xdot = np.array([v*np.cos(theta), v*np.sin(theta), omega, 0, 0])
        return Xdot
    
    @staticmethod
    def F(X, _U):
        """Jacobian of f: matrix of partial derivatives of f (over dX) in the point X, U."""
        # state vector
        _x, _y, theta, v, _omega = X
        # command vector
        #v_u, omega_u = _U
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([
            [0, 0, -v*s, c, 0],
            [0, 0,  v*c, s, 0],
            [0, 0, 0,    0, 1],
            [0, 0, 0,    0, 0],
            [0, 0, 0,    0, 0]
        ])
    
    @staticmethod
    def normalize(angle):
        while angle > np.pi:
            angle -= 2*np.pi
        while angle < -np.pi:
            angle += 2*np.pi
        return angle

    @staticmethod
    def h_lidar(X):
        """Returns what the lidar should output at X."""
        x, y, theta, _v, _omega = X
        return np.array([x, y, theta])
    
    @staticmethod
    def H_lidar(_X):
        return Hl
    
    @staticmethod
    def lidar_residual(z1, z2):
        """Compute the difference between 2 lidar measurements (z1-z2)"""
        x1, y1, theta1 = z1
        x2, y2, theta2 = z2
        return np.array([x1-x2, y1-y2, EkfDiff.normalize(theta1-theta2)])
    
    @staticmethod
    def h_gyro( X):
        """Returns what the gyro should output at X."""
        _x, _y, _theta, _v, omega = X
        return np.array([omega])
    
    @staticmethod
    def H_gyro(_X):
        return Hg
    
    @staticmethod
    def h_enc(X):
        """Returns what the encoders should output at X."""
        _x, _y, _theta, v, omega = X
        return np.array([v, omega])
    
    @staticmethod
    def H_enc(_X):
        return He
    
    @staticmethod
    def h_cam(X):
        """Returns what the encoders should output at X."""
        x, y, theta, _v, _omega = X
        return np.array([x, y, theta])

    @staticmethod
    def H_cam(_X):
        return Hc
    
    @staticmethod
    def h_balise(X, pos_b: Pos):
        """Returns what the encoders should output at X."""
        x, y, theta, _v, _omega = X
        r = np.sqrt((pos_b.x - x)**2 + (pos_b.y - y)**2)
        alpha = np.atan2((pos_b.y - y), (pos_b.x - x)) - theta
        return np.array([r, alpha])
    
    @staticmethod
    def H_balise(X, pos_b: Pos):
        x, y, theta, _v, _omega = X
        r2 = (pos_b.x - x)**2 + (pos_b.y - y)**2
        r = np.sqrt(r2)
        drdx = (x - pos_b.x) / r
        drdy = (y - pos_b.y) / r
        dadx = (pos_b.y - y) / r2
        dady = (x - pos_b.x) / r2
        H = np.array([
            [drdx, drdy,  0, 0, 0],
            [dadx, dady, -1, 0, 0]
        ])
        return H
    
    
    def predict(self, U):
        """
        Propagate the model
        U: commande vector
        """
        self.X = self.X + self.f(self.X, U)*self.dt
        
        F = np.identity(5) + self.F(self.X, U)*self.dt
        self.P = F @ self.P @ F.T + self.Q
        self.send_pos()
    

    def update(self, z, h, Hx:Callable[[np.ndarray], np.ndarray], R, residual=np.subtract):
        """ Update with measures. Since we have multiple sensors, this funtion needs some arguments:
        z: measures from the sensor
        h(X): function returning what the sensor should measure at X.
            If everything is perfect, z = h(X)
        H(x): function returning the jacobian of h(over X) in the point X.
        R: sensor noise
        residual: function that compute the difference between two measurement vector.
            Defaults to substraction, but you may need to provide you own function (i.e. for computing angle difference)
        """
        # y = z - h(x), but in some cases, the minus operation need special handling (i.e. for angles)
        H = Hx(self.X)

        y = residual(z, h(self.X))
        K = self.P @ H.T @ inv(H@self.P@H.T + R)
        self.X = self.X + K@y
        self.P = (np.identity(5) - K@H) @ self.P
        
        self.send_pos()
    

    def handle_lidar_data(self, pub_id : ecal_core.TopicId, msg : ReceiveCallbackData[cpb2.Position]):
        """ Lidar callback. Update the state with lidar measure."""
        z = np.array([msg.message.x, msg.message.y, msg.message.theta])
        self.update(z, self.h_lidar, self.H_lidar, self.Rl, self.lidar_residual)
    
    def handle_gyro(self, pub_id : ecal_core.TopicId, msg : ReceiveCallbackData[cpb2.Ins]):
        """ Gyro callback. Update the state with gyro measure."""
        # TODO vtheta est à l'envers
        z = np.array([msg.message.vtheta])
        self.update(z, self.h_gyro, self.H_gyro, self.Rg)
        
    def handle_encoders(self, pub_id : ecal_core.TopicId, msg : ReceiveCallbackData[cpb2.Speed]):
        """ Encoders callback. Update the state with encoders pos."""
        z = np.array([msg.message.vx, msg.message.vtheta])
        self.update(z, self.h_enc, self.H_enc, self.Re)
    
    def handle_team(self, pub_id : ecal_core.TopicId, msg : ReceiveCallbackData[Side]):
        self.team = msg.message

    def handle_camera(self, pub_id : ecal_core.TopicId, msg : ReceiveCallbackData[Arucos]):
        print(self.team)
        arucos = msg.message.arucos
        if self.team is None:
            return
        elif self.team.color == Side.Color.BLUE:
            arucos_equipe = [a for a in arucos if 1 <= a.ArucoId <= 5]
            print("B")
            print(arucos_equipe)
        elif self.team.color == Side.Color.YELLOW :
            arucos_equipe = [a for a in arucos if 6 <= a.ArucoId <= 10]
            print("J")
            print(arucos_equipe)
        if len(arucos_equipe)== 1:
            arucos_equipe = arucos_equipe[0]
            theta = atan2(
            2 * (arucos_equipe.qw * arucos_equipe.qz + arucos_equipe.qx * arucos_equipe.qy),
            1 - 2 * (arucos_equipe.qy**2 + arucos_equipe.qz**2)
            )
            z = [arucos_equipe.x, arucos_equipe.y, theta] 
            print(z)
            self.update(z, self.h_cam, self.H_cam, self.Rc)

    # def handle_beacons(self, pub_id : ecal_core.TopicId, msg : ReceiveCallbackData[lpb2.Balises]):
    #     # TODO directly in [r, alpha]
    #     # and maybe send beacon by beacon asap, so its not repeated fields (list) anymore
    #     m = msg.message
    #     for i, x, y in zip(m.index, m.x, m.y):
    #         # position at which the beacons has been seen, in the robot frame
    #         r = np.sqrt(x**2 + y**2)
    #         alpha = np.atan2(y, x)
    #         z = np.array([r, alpha])

    #         pos_beacon = self.BEACONS[i]    # theoritical beacon pos
    #         h = partial(self.h_balise, pos_b=pos_beacon)    # get h(X) for this beacon
    #         H = partial(self.H_balise, pos_b=pos_beacon)    # get H(X) for this beacon
    #         self.update(z, h, H, self.Rb)

    
    # def handle_commands(self, pub_id : ecal_core.TopicId, msg : ReceiveCallbackData[cpb2.Speed]):
    #     U = np.array([msg.message.vx, msg.message.vtheta])
    #     self.predict(U)
    
    # def handle_speed_cons(self, pub_id : ecal_core.TopicId, msg : ReceiveCallbackData[cpb2.Speed]):
    #     self.U = np.array([msg.message.vx, msg.message.vtheta])

    def reset_pos(self, pub_id : ecal_core.TopicId, msg : ReceiveCallbackData[cpb2.Position]):
        self.X = np.array([msg.message.x, msg.message.y, msg.message.theta, 0, 0])
    
    def send_pos(self):
        x, y, theta, _, _ = self.X
        self.ekf_pub.send(cpb2.Position(x=x, y=y, theta=theta))
        self.plotX()
    
    def plotX(self):
        x,y,theta,v,omega = self.X
        
        #self.so.sendto(f"pos:{x}:{y}|xy".encode(), TELEPOT_SERVER)
        self.so.sendto(f"theta:{theta}\n".encode(), TELEPOT_SERVER)
        self.so.sendto(f"V:{v}\n".encode(), TELEPOT_SERVER)
        self.so.sendto(f"Omega:{omega}\n".encode(), TELEPOT_SERVER)


def main():
    lidar_var = 5**2, 5**2, 0.004**2
    gyro_var = 0.003**2
    enc_var = 2**2, np.radians(2)**2
    cam_var = 0.5**2, 0.5**2, 0.005**2
    beacons_var = 10**2, np.radians(0.5)**2
    var_speed, var_theta, var_accel = 5**2, 0.1**2, 100**2
    model_var = var_speed, var_speed, var_theta, var_accel, var_accel
    
    X0 = np.array([1500, 1000, 0, 0, 0])

    # prediction rate
    dt = 1/100
    with EkfDiff(X0, dt, lidar_var, gyro_var, enc_var, cam_var, beacons_var, model_var) as ekf:
        while True:
            ekf.predict(np.zeros((2,1)))
            time.sleep(dt)
    
if __name__ == '__main__':
    main()