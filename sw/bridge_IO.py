import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber

import generated.messages_pb2 as message

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

        self.sub_Pano = ProtoSubscriber("IO",message.IO)
        self.sub_Pano.set_callback(self.callback_ecal)

        # a configurer en fonction du branchement sur les pins !!!
        self.pince1 = 1   # servo 1  
        self.pince2 = 2  # servo 2 
        self.pince3 = 3  # servo 3 
        self.pince4 = 4  # servo 4 
        self.bras = 5  # servo 5 
        self.pano = 6  # servo *I2C* 
        self.axL = 7 # ax avec l'ID 5 
        self.axR = 8 # ax avec l'ID 1

        self.serial_port = serial.Serial('/dev/IO',115200) # configurer le port !!!
        self.init_IOs()

        sleep(1) # laissons ecal se réveiller  

    def __repr__(self):

        print(f"\nIO on port: {self.serial_port.name} at rate : {self.serial_port.baudrate}")
        print(f" - Pince_1 \t id : {self.pince1}")
        print(f" - Pince_2 \t id : {self.pince2}")
        print(f" - Pince_3 \t id : {self.pince3}")
        print(f" - Pince_4 \t id : {self.pince4}")        
        print(f" - Bras \t id : {self.bras}")
        print(f" - Pano \t id : {self.pano}")
        print(f" - AXL  \t id : {self.axL}")
        print(f" - AXR  \t id : {self.axR}")
        
        return str('') 
        
        
    def callback_ecal(self,topic_name, msg, timestamp):
        self.send_to_IO(msg.id,msg.val)
    
    def send_to_IO(self,id,val):
        command = str(id) + " " + str(val) + "\n"
        print(bytes(command,"utf-8"))
        #self.serial_port.write(bytes(command,"utf-8"))
   
    def init_IOs(self):
        """Definir ici les valeur a mettre a l'initialisation"""
        print("\nInitialiasing IO")
        self.send_to_IO(self.pince1,1000)
        self.send_to_IO(self.pince2,1000)
        self.send_to_IO(self.pince3,1000)
        self.send_to_IO(self.pince4,1000)
        self.send_to_IO(self.bras,1000)
        self.send_to_IO(self.pano,100)
        print("IO inited !")

if __name__=="__main__":

    carte_IO = IO()
    print(carte_IO)

    while ecal_core.ok():
        pass
