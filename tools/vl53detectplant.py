import numpy as np
import sys
sys.path.append('../generated')
import lidar_data_pb2  as pbl
import robot_state_pb2 as hlm
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher


# Le vl pour la présentation cité de l'espace est le 1

class RadarDetect():
    def __init__(self, nb):
        self.lidar_sub = ProtoSubscriber(f"vl53_{nb}", pbl.Lidar)
        self.lidar_sub.set_callback(self.handle_vl53)
        self.distance_matrix = np.empty((8,8))
        self.eye_pub = ProtoPublisher(f"eye_{nb}",hlm.Eye)

    def handle_vl53(self, topic_name, msg, time):
        self.distances = list(msg.distances)
        distance_matrix = np.empty((8,8))
        def idx(x, y):
            return (7 - y) * 8 + (7 - x)
        for y in range(8):
            for x in range(8):
                distance_matrix[y,x] = self.distances[idx(x,y)]
        self.distance_matrix = distance_matrix
        
        self.x_mins_lines = []
        ys = []
        for y in range(1,6):
            line = self.distance_matrix[y]
            x_mins_before = set()
            x_mins_after = {0,1,2,3,4,5,6,7}
            while x_mins_after != x_mins_before:
                x_mins_before = x_mins_after
                x_mins_after = set()
                for x in x_mins_before:
                    if x == 0:
                        x_mins_after.add(min([x,x+1], key = lambda a: line[a]))
                    elif x == 7:
                        x_mins_after.add(min([x-1,x], key = lambda a: line[a]))
                    else:
                        x_mins_after.add(min([x-1,x,x+1], key = lambda a: line[a]))
            
            to_remove = []
            for x in x_mins_after:
                if line[x] > 200:
                    to_remove.append(x)
            
            for x in to_remove:
                x_mins_after.remove(x)

            if 0 < len(x_mins_after) <= 2:
                ground = False
                if y == 5:
                    for x in x_mins_after:
                        if line[x] > 90: ground = True

                if ground == False:
                    self.x_mins_lines.append(x_mins_after)
                    ys.append(y)
        
        print("#############")
        print(ys)
        print(self.x_mins_lines)
        # for y,x in zip(ys,self.x_mins_lines) :
        #     print(f"y={y} mins={x} val={[self.distance_matrix[y][a] for a in x]}")
        for y in ys:
            print(f"y={y} distances= {self.distance_matrix[y]}")


        if len(self.x_mins_lines) < 3:
            msg = hlm.Eye()
            msg.nombre_pot = 0
            self.eye_pub.send(msg)
            return
        
        single = []
        for i in range(len(self.x_mins_lines)):
            x = self.x_mins_lines[i]
            if len(x) == 1:
                single = [i]
        
        if single:
            print("Single plant")
            number_of_line = len(self.x_mins_lines)
            x_moy = self.x_mins_lines[single[0]].pop()
            distance_moy = self.distance_matrix[ys[single[0]]][x_moy]
            for i in range(number_of_line):
                if i != single[0]:
                    if i in single:
                        x = self.x_mins_lines[i].pop()
                        x_moy += x
                        distance_moy += self.distance_matrix[ys[i]][x]
            x_moy = x_moy / len(single)
            distance_moy = distance_moy / len(single)
            print(f"Position_x: {x_moy} Distance: {distance_moy}")
            msg = hlm.Eye()
            msg.nombre_pot = 1
            msg.distance.append(distance_moy)
            self.eye_pub.send(msg)

        else:
            print("Two plant")
            number_of_line = len(self.x_mins_lines)
            xs_0 = list(self.x_mins_lines[0])
            xs_0.sort()
            x_moy_0 = xs_0[0]
            distance_moy_0 = self.distance_matrix[ys[0]][x_moy_0]
            x_moy_1 = xs_0[1]
            distance_moy_1 = self.distance_matrix[ys[0]][x_moy_1]
            for i in range(1,number_of_line):
                xs = list(self.x_mins_lines[i])
                xs.sort()
                x_moy_0 += xs[0]
                distance_moy_0 += self.distance_matrix[ys[0]][xs[0]]
                x_moy_1 += xs[1]
                distance_moy_1 += self.distance_matrix[ys[0]][xs[1]]
            x_moy_0 = x_moy_0 / number_of_line
            x_moy_1 = x_moy_1 / number_of_line
            distance_moy_0 = distance_moy_0 / number_of_line
            distance_moy_1 = distance_moy_1 / number_of_line
            print(f"Position_x: {x_moy_0} Distance: {distance_moy_0}")
            print(f"Position_x: {x_moy_1} Distance: {distance_moy_1}")
            msg = hlm.Eye()
            msg.nombre_pot = 2
            msg.distance.extend([distance_moy_0, distance_moy_1])
            if abs(distance_moy_0 - distance_moy_1) > 20:
                msg.serie = True
            else:
                msg.serie = False
            self.eye_pub.send(msg)



        
        
        # eye_1234
        # eye_message -> nombre_pot bool=serie distance




            

        

if __name__ == "__main__":
    ecal_core.initialize(sys.argv, "VL53DetectPlant")
    vl_1 = RadarDetect(1)
    while True:
        pass
