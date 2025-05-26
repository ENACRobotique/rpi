from dataclasses import dataclass
from math import cos,sin,sqrt, pi
import numpy as np
import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb
import generated.messages_pb2 as base_pb
import os

@dataclass
class Pos:
    x: float
    y: float
    theta: float

    def __add__(self, other):
        return Pos(self.x + other.x, self.y + other.y, self.theta + other.theta)
    
    def __sub__(self, other):
        return Pos(self.x - other.x, self.y - other.y, normalize_angle(self.theta - other.theta))
    
    def __mul__(self, other):
        if isinstance(other, int) or isinstance(other, float):
            return Pos(self.x*other, self.y*other, self.theta*other)
        raise Exception("Not implemented!")
    
    def copy(self):
        return Pos(self.x, self.y, self.theta)

    def clamp_abs(self, other):
        clamp(-other.x, self.x, other.x)
        x = clamp(-other.x, self.x, other.x)
        y = clamp(-other.y, self.y, other.y)
        theta = clamp(-other.theta, self.theta, other.theta)
        return Pos(x, y, theta)
    
    def cwise_mul(self, other):
        return Pos(self.x*other.x, self.y*other.y, self.theta*other.theta)
    
    def sign(self):
        sx = 1 if self.x >= 0 else -1
        sy = 1 if self.y >= 0 else -1
        stheta = 1 if self.theta >= 0 else -1
        return Pos(sx, sy, stheta)

    def distance(self, other):
        return sqrt((other.x - self.x)**2 + (other.y - self.y)**2)
    
    def norm(self):
        return sqrt(self.x**2 + self.y**2)

    def to_proto(self):
        return robot_pb.Position(x=self.x, y=self.y, theta=self.theta) # type: ignore
    
    @staticmethod
    def from_proto(p: robot_pb.Position):
        return Pos(p.x, p.y, p.theta) # type: ignore

    @staticmethod
    def from_np(p: np.array): # type: ignore
        return Pos(p[0], p[1], p[2])
    
    def to_np(self):
        return np.array([self.x, self.y, self.theta])
    
    def to_frame(self, new_frame):
        """
        pos: position in the current frame
        new_frame: frame that pos will be converted to
        """
        dp = self - new_frame
        ct = cos(new_frame.theta)
        st = sin(new_frame.theta)
        rot = np.array([[ct,  st, 0],
                        [-st, ct, 0],
                        [0,   0,  1]])
        new_pos = rot.dot(np.array([dp.x, dp.y, dp.theta]))
        return Pos.from_np(new_pos)
    
    def from_frame(self, pos_frame):
        """
        pos: position in the pos_frame
        pos_frame: frame 'position' is expressed on, in the current frame
        """
        ct = cos(pos_frame.theta)
        st = sin(pos_frame.theta)
        rot = np.array([[ct,  st, 0],
                        [-st, ct, 0],
                        [0,   0,  1]])
        rot = rot.transpose()
        pos = rot.dot(self.to_np())
        pos = Pos.from_np(pos) + pos_frame
        return pos

@dataclass
class Speed:
    vx: float
    vy: float
    vtheta: float

    def to_proto(self):
        return robot_pb.Speed(vx=self.vx, vy=self.vy, vtheta=self.vtheta) # type: ignore
    
    @staticmethod
    def from_proto(s: robot_pb.Speed):
        return Speed(s.vx, s.vy, s.vtheta) # type: ignore
    
    def xy_norm(self):
        return sqrt(self.vx**2 + self.vy**2)
    
    @staticmethod
    def from_dir(dir_deg: float, speed: float):
        """
        dir_deg: direction in degrees
        speed: speed in m/s
        """
        dir = np.radians(dir_deg)
        vx = speed * cos(dir)
        vy = speed * sin(dir)
        return Speed(vx, vy, 0)

def clamp(lo, val, hi):
    return min(hi, max(val, lo))

def dot(v,w):
    x,y = v
    X,Y = w
    return x*X + y*Y

def length(v):
    x,y, = v
    return sqrt(x*x + y*y)

def vector(b,e):
    x,y = b
    X,Y = e
    return (X-x, Y-y)

def unit(v):
    x,y = v
    mag = length(v)
    return (x/mag, y/mag)

def distance(p0,p1):
    return length(vector(p0,p1))

def scale(v,sc):
    x,y = v
    return (x * sc, y * sc)

def add(v,w):
    x,y = v
    X,Y = w
    return (x+X, y+Y)

def normalize_angle(a):
    while a > pi:
        a -= 2*pi
    while a < -pi:
        a+= 2*pi
    return a

# Given a line with coordinates 'start' and 'end' and the
# coordinates of a point 'pnt' the proc returns the shortest 
# distance from pnt to the line and the coordinates of the 
# nearest point on the line.
#
# 1  Convert the line segment to a vector ('line_vec').
# 2  Create a vector connecting start to pnt ('pnt_vec').
# 3  Find the length of the line vector ('line_len').
# 4  Convert line_vec to a unit vector ('line_unitvec').
# 5  Scale pnt_vec by line_len ('pnt_vec_scaled').
# 6  Get the dot product of line_unitvec and pnt_vec_scaled ('t').
# 7  Ensure t is in the range 0 to 1.
# 8  Use t to get the nearest location on the line to the end
#    of vector pnt_vec_scaled ('nearest').
# 9  Calculate the distance from nearest to pnt_vec_scaled.
# 10 Translate nearest back to the start/end line. 
# Malcolm Kesson 16 Dec 2012

def dist_to_line(pos_pnt:Pos, pos_start:Pos, pos_end:Pos):
    """ Return dist from pos_pnt to the segment [pos_start,pos_end] """
    pnt = (pos_pnt.x,pos_pnt.y)
    start = (pos_start.x,pos_start.y)
    end = (pos_end.x,pos_end.y)
    line_vec = vector(start, end)
    pnt_vec = vector(start, pnt)
    line_len = length(line_vec)
    line_unitvec = unit(line_vec)
    pnt_vec_scaled = scale(pnt_vec, 1.0/line_len)
    t = dot(line_unitvec, pnt_vec_scaled)    
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    nearest = scale(line_vec, t)
    dist = distance(nearest, pnt_vec)
    nearest = add(nearest, start)
    pos_nearest = Pos(x=nearest[0],y=nearest[1],theta=0)
    return (dist, pos_nearest)


def next_path(path_pattern):
    """
    search next available file matching the pattern.
    path_pattern must have one integer format argument: ".*{}.*"
    """
    i = 1
    while os.path.exists(path_pattern.format(i)):
        i += 1
    return path_pattern.format(i)
