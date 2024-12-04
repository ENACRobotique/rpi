#!/usr/bin/env python
import pygame
from pygame.joystick import Joystick
import time
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
import generated.robot_state_pb2 as robot_state_pb2


MAX_SPEED = 300
V_THETA = 1.5

ATTACK3_CONF = {
    "X": 1, #axe
    "X_sens" : -1, #facteur
    "X_offset": 0.,
    "X_dead_zone": 0.1,
    "Y": 0, #axe
    "Y_sens" : -1,
    "Y_offset" : 0.,
    "Y_dead_zone": 0.1,
    "angle_gauche": 3, #bouton
    "angle_droit":4, #bouton
    "vitesse_supra_luminique": 0 #bouton
}

BATTLETRON = {
    "X": 1, #axe
    "X_sens" : -1, #facteur
    "X_offset": 0.15,
    "X_dead_zone": 0.1,
    "Y": 0, #axe
    "Y_sens" : -1,
    "Y_offset" : -0.6,
    "Y_dead_zone": 0.1,
    "angle_gauche": 6, #bouton
    "angle_droit":7, #bouton
    "vitesse_supra_luminique": 11 #bouton
}
class JoystickEcal ():
    def __init__(self):
        self.joystick: Joystick = None
        self.buttons = []
        self.axis = []
        self.conf = None

        ecal_core.initialize([], "Joystick")
        self.publisher = ProtoPublisher("speed_cons", robot_state_pb2.Speed)
        self.message = robot_state_pb2.Speed()
        
    def __repr__(self):
        return f"{len(self.axis)} Axis: {self.axis} \t{len(self.buttons)} Buttons : {self.buttons}"

    def open(self):
        while (self.joystick == None):
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEADDED:
                    self.joystick = pygame.joystick.Joystick(event.device_index)
                    self.axis = [self.axis_get_value(n) for n in range(self.joystick.get_numaxes())]
                    self.buttons = [self.button_get_value(n) for n in range(self.joystick.get_numbuttons())]
                    time.sleep(0.5)
                    print(f"Manette {self.joystick.get_name()} connectée")
    def set_conf(self, conf):
        self.conf = conf


    def button_get_value(self,n):
        return self.joystick.get_button(n)
    
    def axis_get_value(self,n):
            return round(self.joystick.get_axis(n),2)

    def update_value(self):
        for n in range(self.joystick.get_numbuttons()):
            self.buttons[n] = self.button_get_value(n)

        for n in range(self.joystick.get_numaxes()):
            self.axis[n] = self.axis_get_value(n)

    def publish_message(self):
        vx = (MAX_SPEED * self.axis[self.conf["X"]] * self.conf["X_sens"] - self.conf["X_offset"]) * (1 + self.buttons[self.conf["vitesse_supra_luminique"]])
        self.message.vx = vx if abs(self.axis[self.conf["X"]])> self.conf["X_dead_zone"] else 0
        vy = (MAX_SPEED * self.axis[self.conf["Y"]] * self.conf["Y_sens"] - self.conf["Y_offset"]) * (1 + self.buttons[self.conf["vitesse_supra_luminique"]])
        self.message.vy = vy if abs(self.axis[self.conf["Y"]]) > self.conf["Y_dead_zone"] else 0
        self.message.vtheta = V_THETA * (self.buttons[self.conf["angle_gauche"]] - self.buttons[self.conf["angle_droit"]]) * (1 + self.buttons[self.conf["vitesse_supra_luminique"]])
        print(self.message)
        self.publisher.send(self.message)


pygame.init()
pygame.joystick.init()

joysticks_ecal = JoystickEcal()        
joysticks_ecal.open()
joysticks_ecal.set_conf(BATTLETRON)

while True :
    for event in pygame.event.get():
        joysticks_ecal.update_value()
        print(joysticks_ecal)
    joysticks_ecal.publish_message()  
    time.sleep(0.1)
    

