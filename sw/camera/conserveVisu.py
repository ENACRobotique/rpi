#!/usr/bin/env python3
import numpy as np
from sklearn.cluster import DBSCAN
try:
    import matplotlib.pyplot as plt
    visu_available = True
except:
    visu_available = False
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
from generated.robot_state_pb2 import Position_aruco
import time as t
DIM = 500 # affichage matplotlib mm

CAM_DELTA = 100 # écart entre les caméras en mm
CLUSTER_DIST = 40 # mm à régler

CYLINDER_RADIUS = 73/2 # rayon des conserves mm
CONSERVE_ID = 47 # aruco code des conserves
BORDER_DIST = 135
MAX_X = 140 # à régler
TIMEOUT = 1  # second
class visuConserve:
    def __init__(self):
        if not ecal_core.is_initialized():
                ecal_core.initialize(sys.argv, "arucoVisu")

        aruco_sub = ProtoSubscriber("Arucos", Position_aruco) # en mm
        aruco_sub.set_callback(self.getAruco)
        self.MabelArucos = []
        self.DipperArucos = []
        self.DipperClusters = []
        self.MabelClusters = []
        self.last_time = 0

    def getAruco(self, topic_name, msg, time):
        camName = msg.cameraName
        indexes = np.array(msg.index)
        xs = np.array(msg.x)
        ys = np.array(msg.y)
        zs = np.array(msg.z)
        Arucos = np.array(msg.ArucoId)
        self.last_time = t.time()
        
        if camName == "dipper":
            self.DipperArucos = [indexes, xs, ys, zs, Arucos]
            self.DipperClusters = self.arucoCluster(xs, ys, zs, Arucos, CONSERVE_ID)
        else:
            self.MabelArucos = [indexes, xs, ys, zs, Arucos]
            self.MabelClusters = self.arucoCluster(xs, ys, zs, Arucos, CONSERVE_ID)

    def getCylinders(self):
        """Position des centres des cylindres par rapport au centre des deux caméras"""
        camCylinders = {}
        cyl = []
        for cluster in self.DipperClusters:
            x = cluster[0]+CYLINDER_RADIUS-BORDER_DIST
            y = cluster[1]-CAM_DELTA/2
            z = cluster[2]
            cyl.append([x,y,z])
        camCylinders["dipper"] =  cyl
        
        cyl = []
        for cluster in self.MabelClusters:
            x = cluster[0]+CYLINDER_RADIUS-BORDER_DIST
            y = cluster[1]+CAM_DELTA/2
            z = cluster[2]
            cyl.append([x,y,z])
        camCylinders["mabel"] =  cyl
        
        return camCylinders

    def arucoCluster(self, xs, ys, zs, Arucos, filterID):
        points = []
        for i in range(len(Arucos)):
            if Arucos[i] == filterID :
                points.append([xs[i], ys[i], zs[i]])
        points = np.array(points)

        if points.size == 0:
            return np.empty((0, 3))

        if points.ndim == 1:
            points = points.reshape(1, -1)
        # Groupement en cylindres (selon x, y)
        clustering = DBSCAN(eps=CLUSTER_DIST, min_samples=1).fit(points[:, [0, 1]])
        labels = clustering.labels_

        # Moyenne des positions pour chaque cylindre
        cyl_positions = []
        for label in set(labels):
            group = points[labels == label]
            center = group.mean(axis=0)
            cyl_positions.append(center)

        cyl_positions = np.array(cyl_positions)
        return cyl_positions
    
    def initVisualize(self):
        """Call once before calling visualize"""
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(5, 5))
    
    def visualize(self, clusterize = True):
        """Call in a for loop"""
        # Fenêtre persistante de matplotlib
        self.ax.clear()

        # Actualiser la vue matplotlib
        self.ax.set_title("Vue de dessus")
        self.ax.set_xlabel("y (mm)")
        self.ax.set_ylabel("x (mm)")
        self.ax.set_xlim(-DIM/2, DIM/2)
        self.ax.set_ylim(0, DIM)
        self.ax.grid(True)
        self.ax.plot(-CAM_DELTA/2, 0, 'o', label='Dipper', color = "blue")
        self.ax.plot(CAM_DELTA/2, 0, 'o', label='Mabel', color = "pink")
        
        if clusterize :
            for cyl in self.DipperClusters:
                x = cyl[0]
                y = cyl[1]
                self.ax.add_patch(plt.Circle((y-CAM_DELTA/2,x+CYLINDER_RADIUS), CYLINDER_RADIUS, color='blue', alpha=0.5))
            for cyl in self.MabelClusters:
                x = cyl[0]
                y = cyl[1]
                self.ax.add_patch(plt.Circle((y+CAM_DELTA/2,x+CYLINDER_RADIUS), CYLINDER_RADIUS, color='pink', alpha=0.5))

        else :
            # Aruco visibles
            if self.DipperArucos != []:
                xs = self.DipperArucos[1]
                ys = self.DipperArucos[2]
                for i in range(len(xs)):
                    self.ax.add_patch(plt.Circle((ys[i]-CAM_DELTA/2,xs[i]+CYLINDER_RADIUS), CYLINDER_RADIUS, color='blue', alpha=0.5))
            if self.MabelArucos != []:
                xs = self.MabelArucos[1]
                ys = self.MabelArucos[2]
                for i in range(len(xs)):
                    self.ax.add_patch(plt.Circle((ys[i]+CAM_DELTA/2,xs[i]+CYLINDER_RADIUS), CYLINDER_RADIUS, color='pink', alpha=0.5))
        
        self.ax.legend()
        plt.pause(0.001)


    def cam_cons(self):
        """On considère que le robot est bien orienté et plutot loin des conserves!\n
        les bras doivent être relevé sinon on voit rien !!!"""
        if abs(t.time()-self.last_time) > TIMEOUT:
            return None, None
        cyl = self.getCylinders()
        
        dip_cyl = sorted(cyl["dipper"], key=lambda pos: pos[0],) #trier par x
        mab_cyl = sorted(cyl["mabel"], key=lambda pos: pos[0],) #trier par x
        dip_cyl = dip_cyl[:4] # on prend les 4 plus proche en x 
        mab_cyl = mab_cyl[:4] # pour éliminer les objets loins du robot
        dip_cyl = sorted(dip_cyl, key=lambda pos: pos[1],) # trier par y
        mab_cyl = sorted(mab_cyl, key=lambda pos: pos[1], reverse=True) # trier par y à l'envers
        
        all_x = [dip_cyl[i][0] for i in range(len(dip_cyl))] + [mab_cyl[i][0] for i in range(len(mab_cyl))]
        avg_X = np.inf
        if len(all_x):
            avg_X = sum(all_x)/len(all_x)
        # print(avg_X)
        
        ld = len(dip_cyl)
        lm = len(mab_cyl)
        
        easy = (ld == 4 and lm==4) or (ld == 4 and lm==3) or (ld == 3 and lm==4)
        _3_3_chiant = ld == 3 and lm ==3
        _2_3_droite = ld == 2 and lm==3
        _3_2_gauche = ld == 3 and lm==2

        def get_cons(dip_index, mab_index, trust_check=0):
            xd, yd, _ = dip_cyl[dip_index]
            yd = yd+CAM_DELTA/2
            xm, ym, _ = mab_cyl[mab_index]
            ym = ym-CAM_DELTA/2
            xcons = (xm+xd)/2
            ycons = (yd+ym)/2
            if trust_check:
                if abs(abs(yd)-abs(ym)) >= trust_check :
                    if abs(yd)<abs(ym):
                        ycons = ym
                    else:
                        ycons = yd
                    #print("TRUST ISSUES")
            # print(f'dipper:{round(xd,2)}  {round(yd,2)}\tmabel {round(xm,2)}  {round(ym,2)}\tmoy {round(xcons,2)}  {round(ycons,2)}\n')
            return xcons, ycons
        
        def align_closest():
            d_cyl = [(cyl[0],cyl[1]+CAM_DELTA/2,cyl[2]) for cyl in dip_cyl]
            # print(d_cyl)
            m_cyl = [(cyl[0],cyl[1]-CAM_DELTA/2,cyl[2]) for cyl in mab_cyl]
            # print(m_cyl)
            d_i = min(range(len(d_cyl)), key=lambda i: abs(d_cyl[i][1]))
            m_i = min(range(len(m_cyl)), key=lambda i: abs(m_cyl[i][1]))
            return get_cons(d_i, m_i)
        
        xcons, ycons =  None, None
        if avg_X > MAX_X:
            #On est loin des conserves on veut s'aligner principalement selon y
            if easy:
                xcons, ycons = get_cons(1,1) # on prend le 2e en partant du coté désigné
            if _3_3_chiant:
                xcons, ycons = get_cons(1,1,20) # on prend le 2e en partant du coté désigné ou alors on fait confiance à celui qui donne la plus grande distance
            if _2_3_droite:
                xcons, ycons = get_cons(1,0) # on prend le 2e et 1er 
            if _3_2_gauche:
                xcons, ycons = get_cons(0,1) # on prend le 1er et 2e
        else:
            # On es proche des conserve on veut viser celles les plus en face de chaque cam
            xcons, ycons = align_closest()
            # print("Close mode")


        goright = ld == 1 and lm==2
        goleft = ld == 2 and lm==1
        if goright:
            # on va un peu vers la droite
            xcons, ycons = get_cons(0,-1)
            ycons+=100                    
        if goleft:
            # on va un peu vers la gauche
            xcons, ycons = get_cons(-1,0)
            ycons-=100
        
        if xcons is not None and ycons is not None:
            # print(f'cons: {round(xcons,2)}  {round(ycons,2)}\n')
            pass

        
        return xcons, ycons
        



if __name__ == "__main__":
    v = visuConserve()
    if visu_available:
        v.initVisualize()
    while ecal_core.ok():
        if visu_available:
            v.visualize()
        # print(v.getCylinders())
        print(v.cam_cons())