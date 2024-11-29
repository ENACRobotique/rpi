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
    "Y": 0, #axe
    "Y_sens" : -1,
    "angle_gauche": 3, #bouton
    "angle_droit":4, #bouton
    "vitesse_supra_luminique": 0 #bouton
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
        self.message.vx = (MAX_SPEED * self.axis[ATTACK3_CONF["X"]] * ATTACK3_CONF["X_sens"]) * (1 + self.buttons[ATTACK3_CONF["vitesse_supra_luminique"]])
        self.message.vy = MAX_SPEED * self.axis[ATTACK3_CONF["Y"]] * ATTACK3_CONF["Y_sens"] * (1 + self.buttons[ATTACK3_CONF["vitesse_supra_luminique"]])
        self.message.vtheta = V_THETA * (self.buttons[ATTACK3_CONF["angle_gauche"]] - self.buttons[ATTACK3_CONF["angle_droit"]]) * (1 + self.buttons[ATTACK3_CONF["vitesse_supra_luminique"]])
        self.publisher.send(self.message)


pygame.init()
pygame.joystick.init()

joysticks_ecal = JoystickEcal()        
joysticks_ecal.open()



while True :
    for event in pygame.event.get():
        joysticks_ecal.update_value()
        print(joysticks_ecal)
    joysticks_ecal.publish_message()  
    time.sleep(0.1)
    

