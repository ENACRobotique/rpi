import struct
import serial
import threading
from time import time, gmtime, sleep
from enum import Enum
from math import pi

import socket

PORT_NAME = "/dev/bas_niveau" #"COM2" #

def temps_deb (timestamp):
    """Input : t (float) : value given by time()
    Output : a formated string that gives a more explicit time than t
    !!! Cette fonction est à l'heure d'été."""
    itm = gmtime(timestamp+2*3600)
    return '{:04d}/{:02d}/{:02d}\t{:02d}:{:02d}:{:02d}'.format (itm.tm_year, itm.tm_mon,
            itm.tm_mday, itm.tm_hour, itm.tm_min, itm.tm_sec) +'{:.3}'.format (timestamp%1)[1:]


class radioStates(Enum):
    WAITING_FIRST_BYTE=0
    BETWEEN_START_BYTES=1
    WAITING_TYPE=2
    WAITING_REST_OF_NORMAL_MESSAGE=3
    WAITING_REST_OF_STRING_MESSAGE=4

class messageARecevoir(Enum):
    REPORT_POSITION="p"
    REPORT_VITESSE="v"
    STATES_BUTTONS="T"
    CONFIRMATION_ACTION="d"

    MESSAGE_POUR_LOG="M"
    REPORT_CAKE_PRESENCE="C"


class Radio:
    def __init__(self):
        self.PROTOCOL_VERSION =1
        self.continueListening = False
        self.serialObject = serial.Serial (port = PORT_NAME, baudrate=115200, timeout =1)
        self.serial_lock = threading.Lock()
        self.radioState = radioStates.WAITING_FIRST_BYTE
        self.listeningThread = threading.Thread(target=self.listen)
        self.plot_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def startListening (self):
        self.continueListening =True
        self.listeningThread.start()

    def stopListening (self):
        self.continueListening = False

    def __repr__(self):
        return "Radio haut niveau"
    
    def setTargetPosition (self, x, y, theta):
        """Sends a position command to the low level program.
        x, y, and theta are floats.
        x and y are in meters
        theta is in radians"""
        message = b'p'+struct.pack("fff",x,y,theta)
        self.sendMessage(message)
    
    def resetPosition (self, x, y, theta):
        """Resets the position of the low level program.
        x, y, and theta are floats.
        x and y are in meters
        theta is in radians"""
        message = b'r'+struct.pack("fff",x,y,theta)
        self.sendMessage(message)
    
    def sendStopSignal (self):
        """Sends a command to stop the robot."""
        self.sendMessage(b'S')

    def sendSlowDownSignal (self):
        """Sends a command to slow the robot down."""
        self.sendMessage(b's')
        
    def sendResumeSignal (self):
        """Sends a command to make the robot ignore the last stop or slow command."""
        self.sendMessage(b'R')

    def sendCostumeSignal (self):
        """Sends a command that will activate the end of match sequence for the robot."""
        self.sendMessage(b'D')
    
    def sendClawSignal (self,claw_pos):
        """Send a command to the claws of the robot
        valueClaws is one char converted to an int.
        Accepted values are :
            'o' : open
            'g' : grab
            'c' : closed
            'C' : check cake presence
        """
        message = b'g'+struct.pack("B",claw_pos)
        self.sendMessage(message)

    def sendTurbineSignal(self,valueTurbine):
        """Send a command to turn the turbine on or off.
        valueTurbine is a char
        Accepted values are :
            'a' : allumé
            'e" : éteint
        /!\\ Deprecated AF, there's no turbine on robot. That code will be removed at the next update
        """
        if valueTurbine in "ae":
            message = b't'+struct.pack("c",valueTurbine)
            self.sendMessage(message)
        else :
            print("Invalid value for turbine : {}".format(valueTurbine))

    def sendTobogganSignal (self, valueToboggan):
        """Send a command to turn the slide on or off.
        valueToboggan is a char converted to an int
        Accepted values are :
            'r' : rentré
            's' : sorti
        """
        message = b'T'+struct.pack("B",valueToboggan)
        self.sendMessage(message)

    def sendPointDisplay (self,pointNumber):
        """Sends a number to be displayed by the display
        pointNumber should be an integer smaller than 255
        """
        message =b'P'+struct.pack("B",pointNumber%256)
        self.sendMessage(message)

    def sendStoreDiscsInsideSignal(self,positionToStore,numSequence):
        """Sends a signal to ask the robot to pick disks and store them.
        The robot will tell when it is done by sending an ack signal.
            postionToStore to store is an int. Accepted values are : 1, 3 and 5
            
            numSequence is an arbitrary int to increment"""
        if positionToStore in [1,3,5]:
            message=b'a'+struct.pack("BB",positionToStore,numSequence)
            self.sendMessage(message)
    
    def sendPicDiscFromStorage(self,positionInStore,numSequence):
        """Sends a signal to ask the robot to pick a disk in storage and place it on the table.
        The robot will tell when it is done by sending an ack signal.
            postionInStore to store is an int. Accepted values are : 1, 3 and 5

            numSequence is an arbitrary int to increment"""
        if positionInStore in [1,3,5]:
            message=b'd'+struct.pack("BB",positionInStore,numSequence)
            self.sendMessage(message)

    def verifyAndExec(self,byteArray,typeReçu):
        match typeReçu:
            case messageARecevoir.REPORT_POSITION:
                (x,y,theta,chksum)=struct.unpack("fffB",byteArray)
                sum = self.PROTOCOL_VERSION + ord('p')
                for byte in byteArray[:-1]:
                    sum+=byte
                if sum%256 == chksum:
                    #print ("success : Pos ({}, {}, {})\n\n".format(x,y,theta))

                    self.handle_pos_report(x,y,theta)
                    
                else:
                    print("FAILED CHECKSUM : MessageError")
                    print(b'p'+byteArray)
            case messageARecevoir.REPORT_VITESSE:
                (Vx,Vy,Vtheta,chksum)=struct.unpack("fffB",byteArray)
                sum = self.PROTOCOL_VERSION + ord('v')
                for byte in byteArray[:-1]:
                    sum+=byte
                if sum%256 == chksum:
                    #print ("success : Speed ({}, {}, {})\n\n".format(Vx,Vy,Vtheta))

                    self.handle_speed_report(Vx,Vy,Vtheta)

                else:
                    print("FAILED CHECKSUM : MessageError")
                    print(b'v'+byteArray)
            case messageARecevoir.STATES_BUTTONS:
                (tir,col,posDep,chksum,)=struct.unpack("BBBB",byteArray)
                if chksum == (ord('T')+tir+col+posDep+self.PROTOCOL_VERSION) % 256:                 
                    self.handle_IHM(tir, col, posDep)

                else:
                    print("FAILED CHECKSUM : MessageError")
                    print(b'T'+byteArray)
            case messageARecevoir.CONFIRMATION_ACTION:
                (num,chksum)=struct.unpack("BB",byteArray)
                sum = self.PROTOCOL_VERSION + ord('d') +byteArray[0]
                if sum%256 == chksum:
                    print ("success : Action Confirmed : {}\n\n".format(num))
                    self.handle_action_report(num)

                else:
                    print("FAILED CHECKSUM : MessageError")
                    print(b'd'+byteArray)
            
            case messageARecevoir.REPORT_CAKE_PRESENCE:
                (is_cake,chksum)=struct.unpack("BB",byteArray)
                sum = self.PROTOCOL_VERSION + ord('C')
                for byte in byteArray[:-1]:
                    sum+=byte
                if sum%256 == chksum:
                    self.handle_cake_presence_report(is_cake)
                else:
                    print("FAILED CHECKSUM : MessageError")
                    print(b'C'+byteArray)

    def sendMessage(self,dataByteString):
        sum = self.PROTOCOL_VERSION
        for i in dataByteString:
            sum+=i
        message=b"\n\n"+dataByteString+struct.pack("B",sum%256)
        self.serial_lock.acquire()
        self.serialObject.write(message)
        self.serial_lock.release()


    def listen(self):
        print("Starting Listening")
        numberOfExpectedBytes =0
        typeReçu =None
        bytesMessage =b""
        c=None
        while self.continueListening:
            sleep(0.0002)#pour éviter de bloquer le processeur
            match self.radioState:
                case radioStates.WAITING_FIRST_BYTE:
                    while (c!='\n') and (self.serialObject.in_waiting !=0):
                        c=chr(struct.unpack("B",self.serialObject.read(1))[0])
                    if c=='\n':
                        self.radioState=radioStates.BETWEEN_START_BYTES
                case radioStates.BETWEEN_START_BYTES:
                    if self.serialObject.in_waiting !=0:
                        c=chr(struct.unpack("B",self.serialObject.read(1))[0])
                        self.radioState = radioStates.WAITING_TYPE if c=='\n' else radioStates.WAITING_FIRST_BYTE
                case radioStates.WAITING_TYPE:
                    if self.serialObject.in_waiting !=0:
                        c=chr(struct.unpack("B",self.serialObject.read(1))[0])
                        match c:
                            case messageARecevoir.REPORT_POSITION.value:
                                numberOfExpectedBytes=13#3 floats(4o) + 1o checksum
                                typeReçu = messageARecevoir.REPORT_POSITION
                                self.radioState=radioStates.WAITING_REST_OF_NORMAL_MESSAGE
                            case messageARecevoir.REPORT_VITESSE.value:
                                numberOfExpectedBytes=13#3 floats(4o) + 1o checksum
                                typeReçu = messageARecevoir.REPORT_VITESSE
                                self.radioState=radioStates.WAITING_REST_OF_NORMAL_MESSAGE
                            case messageARecevoir.STATES_BUTTONS.value:
                                numberOfExpectedBytes=4#1o checksum
                                typeReçu = messageARecevoir.STATES_BUTTONS
                                self.radioState=radioStates.WAITING_REST_OF_NORMAL_MESSAGE
                            case messageARecevoir.CONFIRMATION_ACTION.value:
                                numberOfExpectedBytes=2#1o numAction + 1o checksum
                                typeReçu = messageARecevoir.CONFIRMATION_ACTION
                                self.radioState=radioStates.WAITING_REST_OF_NORMAL_MESSAGE
                            case messageARecevoir.MESSAGE_POUR_LOG.value:
                                self.radioState=radioStates.WAITING_REST_OF_STRING_MESSAGE
                                bytesMessage =b''
                            case messageARecevoir.REPORT_CAKE_PRESENCE.value:
                                numberOfExpectedBytes= 2
                                typeReçu = messageARecevoir.REPORT_CAKE_PRESENCE
                                self.radioState=radioStates.WAITING_REST_OF_NORMAL_MESSAGE

                            case '\n':
                                pass
                            case _:
                                self.radioState=radioStates.WAITING_FIRST_BYTE
                case radioStates.WAITING_REST_OF_NORMAL_MESSAGE:
                    if self.serialObject.in_waiting >= numberOfExpectedBytes:
                        self.verifyAndExec(self.serialObject.read(numberOfExpectedBytes),typeReçu)
                        self.radioState=radioStates.WAITING_FIRST_BYTE
                    
                case radioStates.WAITING_REST_OF_STRING_MESSAGE:
                    while (c!=b'\n') and (self.serialObject.in_waiting !=0):
                        c=self.serialObject.read(1)
                        bytesMessage+=c
                    if (c==b'\n'):
                        tStampString = temps_deb(time())
                        try :
                            ...
                            msg = bytesMessage.decode()
                            self.handle_message(msg)
                        except Exception:
                            print("error decoding string message")

                        self.radioState=radioStates.WAITING_FIRST_BYTE

    def handle_pos_report(self,x,y,theta):
        print(f"\t\t\t\t\t  x = {round(x,3)} y = {round(y,3)} yaw = {round(theta*180/pi,3)}, ")

    def handle_speed_report(self,vx,vy,vtheta):
        pass

    
    def handle_match_report(self):
        print("Handle_match_report Unimplemented")

    def handle_action_report(self,num):
        print("Handle_action_report Unimplemented")

    def handle_message(self,msg):
        print("Handle_message Unimplemented")
    
    def handle_checksum_error(self):
        print("Handle_checksum Unimplemented")

    def handle_cake_presence_report(self,is_cake):
        print("handle_cake_presence_report Unimplemented")
    
    def handle_IHM(self, tirette, color, posdep):
        print("handle_IHM Unimplemented")
    


if __name__=="__main__":
    radio=Radio()
    radio.startListening()
    i=1
    
    while True:
        sleep(0.2)
        radio.sendPointDisplay(i)
        #print("send "+str(i%256))
        i+=1
