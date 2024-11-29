import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber
import sys
sys.path.append('../generated')

import robot_state_pb2 as robot_pb

import sys
import serial
from time import sleep

# Binding avec la carte 
 ########################
#   id   # Port carte IO #
#   1    #    servo 1    #
#   2    #    servo 2    #
#   3    #    servo 3    #
#   4    #    servo 4    #
#   5    #    servo 5    #
#   6    #    servo I2C  #
#   7    #    ax 5       #
#   8    #    ax 1       #
 ########################
 # Si on parle a l'id 1 on parle
 # au composant branché sur le port servo 1 écrit sur la carte !!!

class IO:
    def __init__(self):

        
        ecal_core.initialize(sys.argv,"Bridge IO")

        self.sub_Pano = ProtoSubscriber("Actionneur",robot_pb.IO)
        self.sub_Pano.set_callback(self.callback_ecal)
        self.serial_port = serial.Serial('/dev/robot_io',115200) # configurer le port !!!
        self.init_IOs()

        sleep(1) # laissons ecal se réveiller  

    def __repr__(self):
        print(f"\nIO on port: {self.serial_port.name} at rate : {self.serial_port.baudrate}")
        return str('') 
        
        
    def callback_ecal(self,topic_name, msg, timestamp):
        self.send_to_IO(msg.id,msg.val)
    
    def send_to_IO(self,id,val):
        command = str(id) + " " + str(val) + "\n"
        print(bytes(command,"ascii"))
        self.serial_port.write(bytes(command,"ascii"))
    
    def init_IOs(self):
        """Definir ici les valeur a mettre a l'initialisation"""
        print("\nInitialiasing IO")

        print("IO inited !")

if __name__=="__main__":

    carte_IO = IO()
    print(carte_IO)

    while ecal_core.ok():
        sleep(1)
