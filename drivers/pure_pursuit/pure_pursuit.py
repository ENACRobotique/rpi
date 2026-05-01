import time
import numpy as np
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Publisher as ProtoPublisher
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
import sys
sys.path.append("../..")
import generated.common_pb2 as common_pb
from ecal.msg.common.core import ReceiveCallbackData
from common import Pos,Speed

LOOK_AHEAD_DIST = 150
DIST_ACURACY = 10
THETA_ACCURACY = 5*np.pi/180

VMAX = 600; # mm/s
OMG_MAX = np.pi # rad/s
Ke = 3 # pente de décélaration (probablement empirique)
Ke_ANG = 3*np.pi
A = 10000; # pente d'accélération 
A_ANG = 75*np.pi; # pente d'accélération angulaire


class PurePursuitEcal:
    def __init__(self):
        if not ecal_core.is_initialized():
            ecal_core.initialize("PurePursuit")
        time.sleep(1)

        self.last_time = time.time()
        self.dt = 0

        #Subscriber Trajectoire
        self.traj = []
        self.nbseg = 0
        self.indice_seg = 0
        self.traj_sub = ProtoSubscriber(common_pb.Trajectoire,"set_trajectoire")
        self.traj_sub.set_receive_callback(self.get_traj)
        
        #Subscriber Speed
        self.speed_robot = Speed(0,0,0)

        #Subscriber Position
        self.pos_robot = Pos(0,0,0)
        self.pos_sub = ProtoSubscriber(common_pb.Position,"odom_pos")
        self.pos_sub.set_receive_callback(self.get_robot_pos)

        #Publisher
        self.speed_pub = ProtoPublisher(common_pb.Speed, "speed_cons")


    def get_traj(self, pub_id : ecal_core.TopicId, data : ReceiveCallbackData[common_pb.Trajectoire]):
        self.traj = [Pos.from_proto(pos) for pos in data.message.pos]
        self.nbseg = len(self.traj)-1
        self.indice_seg = 0
        print(self.traj)
    
    def get_robot_pos(self, pub_id : ecal_core.TopicId, data : ReceiveCallbackData[common_pb.Position]):
        # TODO
        # La ligne suivante ne fait rien puisque que la valeur de retour de la methode statique (méthode de classe) n'est pas utilisée !
        # self.pos_robot.from_proto(data.message)
        # Il faut plutôt faire comme ça :
        self.pos_robot = Pos.from_proto(data.message)

    def publish_speed_cons(self,vx,vy,vtheta):
        message = common_pb.Speed()
        message.vx = vx
        message.vy = vy
        message.vtheta = vtheta
        self.speed_pub.send(message)

    def compute_speed_cons(self, target_w: Pos):
        # TODO
        # la méthode to_frame ne mute pas la position, elle renvoie un nouvelle position !
        # donc la ligne suivant ne fait rien...
        # target.to_frame(self.pos_robot)
        # Il faut utiliser la valeur de retour
        target_r = target_w.to_frame(self.pos_robot)
        dist_theta = target_r.gisement() if target_w != Pos(self.pos_robot.x,self.pos_robot.y, self.traj[-1].theta) else self.traj[-1].theta - self.pos_robot.theta
        alpha_courbe = abs(target_r.y)/target_r.norm() if target_r.norm() !=0 else 1 #Idée 1 : Ralentir quand on tourne

        # Calcul vx
        if target_r.x < 0 :
            speed_cons_d = max([- Ke * target_r.norm(), -VMAX ]) # , self.speed_robot.vx - A * self.dt])
        else :
            speed_cons_d = min([Ke * target_r.norm(), VMAX])  #, self.speed_robot.vx + A * self.dt])

        # Calcul vtheta
        #sudo vote_RN
        if dist_theta < 0:
            speed_cons_theta = max([Ke_ANG * dist_theta, -OMG_MAX ]) #, self.speed_robot.vtheta - A_ANG * self.dt])
        else :
            speed_cons_theta = min([Ke_ANG * dist_theta, OMG_MAX ]) #, self.speed_robot.vtheta + A_ANG * self.dt])

        self.speed_robot = Speed((1-alpha_courbe) * speed_cons_d,0,alpha_courbe * speed_cons_theta) 
        self.publish_speed_cons((1-alpha_courbe) *speed_cons_d,0,alpha_courbe * speed_cons_theta)
        self.last_time = time.time()
    

    def update(self):
        if len(self.traj) > 0:
            # TODO
            # Pourquoi en attribut de la classe ? self.dt => dt
            # Et attention à la première fois => réinitialiser self.last_time = time.time() dans self.get_traj()
            # time.time() est en secondes, donc pas besoin de diviser par 1000
            self.dt = (time.time() - self.last_time)/1000

            if self.indice_seg == self.nbseg-1: #Si on est dans le dernier segment
                dist_theta = self.traj[-1].theta - self.pos_robot.theta
                if self.pos_robot.distance(self.traj[-1]) < DIST_ACURACY :
                    if abs(dist_theta) < THETA_ACCURACY:
                        self.publish_speed_cons(0,0,0)
                        self.traj = []
                        self.indice_seg = 0
                        print("FINISH at ", self.pos_robot," WOUHOU")
                    else :
                        self.compute_speed_cons(Pos(self.pos_robot.x,self.pos_robot.y, self.traj[-1].theta))
                else:
                    self.compute_speed_cons(self.traj[-1])
            
            else :
                ################### Test pour savoir si on intersecte avec le prochain segment
                p1 = self.traj[self.indice_seg+1] - self.pos_robot
                p2 = self.traj[self.indice_seg+2] - self.pos_robot
                d = p2 - p1
                d_r = d.norm()
                D =  p1.x * p2.y - p2.x * p1.y

                discriminant = (LOOK_AHEAD_DIST * d_r)**2 - D**2

                if discriminant >= 0 :
                    print("segment suivant : ", self.indice_seg+1)
                    self.indice_seg +=1
                ################## fin du test ##################################

                p1 = self.traj[self.indice_seg] - self.pos_robot 
                p2 = self.traj[self.indice_seg+1] - self.pos_robot
                d = p2 - p1
                # d_x = p2.x - p1.x
                # d_y = p2.y - p1.y
                # d_r = np.sqrt(d_x**2 + d_y**2)
                d_r = d.norm()
                D =  p1.x * p2.y - p2.x * p1.y

                discriminant = (LOOK_AHEAD_DIST * d_r)**2 - D**2

                if discriminant < 0 : # Pas d'intersection trouvée, on retourne au premier point du segment
                    self.compute_speed_cons(p1 + self.pos_robot)

                elif discriminant == 0 : # On est tangent
                    target = Pos(D * d.y / d_r**2,-D * d.x / d_r**2,self.traj[self.indice_seg].theta)
                    self.compute_speed_cons(target + self.pos_robot)

                elif discriminant > 0 : # 2 intersections trouvées
                    sol1 = Pos((D * d.y + d.sign().y * d.x * np.sqrt(discriminant))/d_r**2,(-D * d.x + abs(d.y) * np.sqrt(discriminant))/d_r**2,self.traj[self.indice_seg].theta)
                    sol2 = Pos((D * d.y - d.sign().y * d.x * np.sqrt(discriminant))/d_r**2,(-D * d.x - abs(d.y) * np.sqrt(discriminant))/d_r**2,self.traj[self.indice_seg].theta)
                    if p2.distance(sol1) < p2.distance(sol2):
                        self.compute_speed_cons(sol1 + self.pos_robot)
                    else : 
                        self.compute_speed_cons(sol2 + self.pos_robot)


if __name__ == '__main__':
    purepursuit_ecal = PurePursuitEcal()
    while True :
        purepursuit_ecal.update()
        time.sleep(0.1)