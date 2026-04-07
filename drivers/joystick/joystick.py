#!/usr/bin/env python3
import pygame
from pygame.joystick import Joystick
import time
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Publisher as ProtoPublisher
import sys
sys.path.append("../..")
# import generated.robot_state_pb2 as robot_state_pb2
import generated.common_pb2 as common_pb
# import generated.messages_pb2 as message_pb2
from sw.IO.actionneurs import IO_Manager
from sw.IO.actionneurs import PosTentacle
from sw.IO.actionneurs import POMPES_DROITES
from sw.IO.actionneurs import POMPES_GAUCHES
from joystick_confs import *

MAX_SPEED = 300
V_THETA = 2

FRAME_W = 1
FRAME_R = 0


TIMEOUT_COMMAND = 1
PLANCHE = 0
RENTREUR = 1
ASSCENSEUR = 2
IDLE = 0
GLISSING = 1
class JoystickEcal ():
    def __init__(self):
        self.joystick: Joystick = None
        self.alpha = 0.3
        self.buttons = []
        self.axis = []
        self.hats = []
        self.conf = BATTLETRON

        # Actionneur 2025
        self.glisseMode = PLANCHE
        self.glisseState = IDLE
        self.count_zero = 0
        ########

        # Actionneur 2026
        self.posG = PosTentacle.BAS
        self.posD = PosTentacle.BAS
        self.pumpG = False
        self.pumpGList = [False for k in range(4)]
        self.pumpD = False
        self.pumpDList = [False for k in range(4)]

        if not ecal_core.is_initialized():
            ecal_core.initialize("Joystick")
        time.sleep(1) # on laisse ecal se reveiller

        self.speed_publisher = ProtoPublisher(common_pb.Speed, "speed_cons")
        self.message = common_pb.Speed()
        self.IO_manager = IO_Manager()

        self.verrouLock = False
        self.brasUP = False
        self.grabHaut = False
        self.grabBas = False
        
    def __repr__(self):
        return f"{len(self.axis)} Axis: {self.axis} \t{len(self.buttons)} Buttons : {self.buttons} \t{len(self.hats)} Hats : {self.hats}"

    def open(self):
        while (self.joystick == None):
            time.sleep(0.1)
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEADDED:
                    self.joystick = pygame.joystick.Joystick(event.device_index)
                    self.axis = [self.axis_get_value(n) for n in range(self.joystick.get_numaxes())]
                    self.buttons = [self.button_get_value(n) for n in range(self.joystick.get_numbuttons())]
                    self.hats = [self.hats_get_value(n) for n in range(self.joystick.get_numhats())]
                    time.sleep(0.5)
                    print(f"Manette {self.joystick.get_name()} connectée")
    def set_conf(self, conf):
        self.conf = conf


    def button_get_value(self,n):
        return self.joystick.get_button(n)
    
    def axis_get_value(self,n):
            return round(self.joystick.get_axis(n),2)
    
    def hats_get_value(self,n):
            return self.joystick.get_hat(n)

    def update_value(self):
        for n in range(self.joystick.get_numbuttons()):
            self.buttons[n] = self.button_get_value(n)

        for n in range(self.joystick.get_numaxes()):
            self.axis[n] = self.axis_get_value(n)
        
        for n in range(self.joystick.get_numhats()):
            self.hats[n] = self.hats_get_value(n)

    def publish_command(self):
        vx = (MAX_SPEED * self.axis[self.conf["X"]] * self.conf["X_sens"] - self.conf["X_offset"]) * (1 + self.buttons[self.conf["vitesse_supra_luminique"]])

        self.message.vx = vx * self.alpha + (1-self.alpha) * self.message.vx if abs(self.message.vx) < abs(vx) else vx  # vx if abs(self.axis[self.conf["X"]])> self.conf["X_dead_zone"] else 0
        vy = (MAX_SPEED * self.axis[self.conf["Y"]] * self.conf["Y_sens"] - self.conf["Y_offset"]) * (1 + self.buttons[self.conf["vitesse_supra_luminique"]])
        self.message.vy = vy if abs(self.axis[self.conf["Y"]]) > self.conf["Y_dead_zone"] else 0
        
        if self.conf == BATTLETRON:
            vtheta = (V_THETA * self.axis[self.conf["THETA"]] * self.conf["THETA_sens"] - self.conf["THETA_offset"]) * (1 + self.buttons[self.conf["theta_supra_luminique"]])
            self.message.vtheta = vtheta if abs(self.axis[self.conf["THETA"]]) > self.conf["THETA_dead_zone"] else 0

            # Actionneur 2026
            if self.buttons[self.conf["gachette_gauche"]] == 1:
                if self.posG == PosTentacle.HAUT:
                    if self.pumpG == True:
                        self.posG =  PosTentacle.DROP
                    else:
                        self.posG = PosTentacle.BAS
                else :
                    self.posG = PosTentacle.HAUT
                self.IO_manager.moveG(self.posG)
                time.sleep(0.25)

            if self.buttons[self.conf["L1"]] == 1:
                self.pumpG = not self.pumpG
                self.pumpGList = [self.pumpG for pump in self.pumpGList]
                self.IO_manager.GrabG(self.pumpG)
                time.sleep(0.1)

            if self.buttons[self.conf["R1"]] == 1:
                self.pumpD = not self.pumpD
                self.pumpDList = [self.pumpD for pump in self.pumpDList]
                self.IO_manager.GrabD(self.pumpD)
                time.sleep(0.1)


            if self.buttons[self.conf["gachette_droite"]] == 1:
                if self.posD == PosTentacle.HAUT:
                    if self.pumpD == True:
                        self.posD =  PosTentacle.DROP
                    else:
                        self.posD = PosTentacle.BAS
                else :
                    self.posD = PosTentacle.HAUT
                self.IO_manager.moveD(self.posD)
                time.sleep(0.1)

            if self.buttons[self.conf["option"]] == 1:
                self.IO_manager.moveD(PosTentacle.RETOURNE)
                time.sleep(0.1)
            
            if self.buttons[self.conf["share"]] == 1:
                self.IO_manager.moveG(PosTentacle.RETOURNE)
                time.sleep(0.1)

            if self.buttons[self.conf["triangle"]] == 1:
                self.pumpDList[0] = not self.pumpDList[0]
                self.IO_manager.Grab(POMPES_DROITES[0],self.pumpDList[0])
                time.sleep(0.1)

            if self.buttons[self.conf["rond"]] == 1:
                self.pumpDList[1] = not self.pumpDList[1]
                self.IO_manager.Grab(POMPES_DROITES[1],self.pumpDList[1])
                time.sleep(0.1)

            if self.buttons[self.conf["croix"]] == 1:
                self.pumpDList[2] = not self.pumpDList[2]
                self.IO_manager.Grab(POMPES_DROITES[2],self.pumpDList[2])
                time.sleep(0.1)

            if self.buttons[self.conf["carre"]] == 1:
                self.pumpDList[3] = not self.pumpDList[3]
                self.IO_manager.Grab(POMPES_DROITES[3],self.pumpDList[3])
                time.sleep(0.1)

            if self.hats[0] == (0,1):
                self.pumpGList[0] = not self.pumpGList[0]
                self.IO_manager.Grab(POMPES_GAUCHES[0],self.pumpGList[0])
                time.sleep(0.1)

            if self.hats[0] == (1,0):
                self.pumpGList[1] = not self.pumpGList[1]
                self.IO_manager.Grab(POMPES_GAUCHES[1],self.pumpGList[1])
                time.sleep(0.1)

            if self.hats[0] == (0,-1):
                self.pumpGList[2] = not self.pumpGList[2]
                self.IO_manager.Grab(POMPES_GAUCHES[2],self.pumpGList[2])
                time.sleep(0.1)

            if self.hats[0] == (-1,0):
                self.pumpGList[3] = not self.pumpGList[3]
                self.IO_manager.Grab(POMPES_GAUCHES[3],self.pumpGList[3])
                time.sleep(0.1)
                
                


        if self.conf == ATTACK3_CONF:
            self.message.vtheta = V_THETA * (self.buttons[self.conf["angle_gauche"]] - self.buttons[self.conf["angle_droit"]]) * (1 + self.buttons[self.conf["vitesse_supra_luminique"]])
        
        if self.message.vx == 0 and self.message.vy == 0 and self.message.vtheta == 0:
            self.count_zero += 1
        else:
            self.count_zero = 0
        
        if self.count_zero < 10:
            self.speed_publisher.send(self.message)
        

if __name__ == '__main__':
    pygame.init()
    pygame.joystick.init()

    joysticks_ecal = JoystickEcal()        
    joysticks_ecal.open()
    joysticks_ecal.set_conf(BATTLETRON)

    while True :
        pygame.event.get()
        joysticks_ecal.update_value()
        joysticks_ecal.publish_command()
        time.sleep(0.1)
    