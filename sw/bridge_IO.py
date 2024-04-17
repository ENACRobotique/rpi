import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber

import generated.messages_pb2 as message

import sys
import serial
from time import sleep


class IO:
    def __init__(self):

        self.serial_port = serial.Serial('/dev/IO',115200) # configurer le port !!!
        
        ecal_core.initialize(sys.argv,"Bridge IO")

        self.sub_Pano = ProtoSubscriber("pano",message.IO)
        self.sub_Pano.set_callback(self.send_to_IO)
        
        self.sub_Pince = ProtoSubscriber("pinces",message.IO)
        self.sub_Pince.set_callback(self.send_to_IO)
        
        self.sub_AX = ProtoSubscriber("AX",message.IO)
        self.sub_AX.set_callback(self.send_to_IO)


        # a configurer en fonction du branchement sur les pins !!!
        self.pince1 = 1   # servo 1  
        self.pince2 = 2  # servo 2 
        self.pince3 = 3  # servo 3 
        self.pince4 = 4  # servo 4 
        self.bras = 5  # servo 5 
        self.pano = 6  # servo *I2C* 
        self.axL = 7 # ax avec l'ID 5 
        self.axR = 8 # ax avec l'ID 1

        sleep(1) # laissons ecal se réveiller  

    def __repr__(self):

        print(f"IO on port: {self.serial_port.name} at rate : {self.serial_port.baudrate}\n")
        print(f"     Pince 1 {self.pince1}\n")
        print(f"Pince 2 {self.pince2}\n")
        print(f"Pince 3 {self.pince3}\n")
        print(f"Pince 4 {self.pince4}\n")        
        print(f"Pano {self.pano}\n")
        print(f"Bras {self.bras}\n")
        print(f"AXL {self.axL}\n")
        print(f"AXR {self.axR}\n")
        
        

    def send_to_IO(self,topic_name, msg, timestamp):
        command = str(msg.id) + " " + str(msg.val) + "\n"
        self.serial_port.write(bytes(command,"utf-8"))
    
    def init_IOs(self):
        pass




        