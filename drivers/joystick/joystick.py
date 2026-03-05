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
        self.buttons = []
        self.axis = []
        self.hats = []
        self.conf = BATTLETRON
        self.glisseMode = PLANCHE
        self.glisseState = IDLE
        self.count_zero = 0
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
        self.message.vx = vx if abs(self.axis[self.conf["X"]])> self.conf["X_dead_zone"] else 0
        vy = (MAX_SPEED * self.axis[self.conf["Y"]] * self.conf["Y_sens"] - self.conf["Y_offset"]) * (1 + self.buttons[self.conf["vitesse_supra_luminique"]])
        self.message.vy = vy if abs(self.axis[self.conf["Y"]]) > self.conf["Y_dead_zone"] else 0
        
        if self.conf == BATTLETRON:
            vtheta = (V_THETA * self.axis[self.conf["THETA"]] * self.conf["THETA_sens"] - self.conf["THETA_offset"]) * (1 + self.buttons[self.conf["theta_supra_luminique"]])
            self.message.vtheta = vtheta if abs(self.axis[self.conf["THETA"]]) > self.conf["THETA_dead_zone"] else 0
            
            #Actionneur 2025
            if self.hats[0][self.conf["selectGlisse"]] != 0:
                self.glisseMode +=self.hats[0][self.conf["selectGlisse"]]
                if self.glisseMode >2 :
                    self.glisseMode = 2
                if self.glisseMode <0 :
                    self.glisseMode = 0
                time.sleep(0.25)
            
            if self.buttons[self.conf["verrou"]] == 1:
                self.verrouLock = not self.verrouLock
                self.IO_manager.calibrateLift()
                time.sleep(0.25)

            if self.buttons[self.conf["bras"]] == 1:
                # self.brasUP = not self.brasUP
                self.IO_manager.deployMacon()
                time.sleep(0.25)

            if self.buttons[self.conf["grabHaut"]] == 1:
                self.grabHaut = not self.grabHaut
                self.IO_manager.grabHighConserve(self.grabHaut)
                time.sleep(0.25)

            if self.buttons[self.conf["grabBas"]] == 1:
                self.grabBas = not self.grabBas
                self.IO_manager.grabLowConserve(self.grabBas)
                time.sleep(0.25)
                
            if self.buttons[self.conf["action_1"]] == 1:
                self.glisseMode = PLANCHE
                #self.IO_manager.ramasseGradin()
            
            if self.buttons[self.conf["action_2"]] == 1:
                self.glisseMode = PLANCHE
                #self.IO_manager.construitGradin()
            
            

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
    