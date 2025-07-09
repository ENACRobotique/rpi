#!/usr/bin/env python3
import pygame
from pygame.joystick import Joystick
import time
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
import sys
sys.path.append("../..")
# import generated.robot_state_pb2 as robot_state_pb2
import generated.common_pb2 as common_pb
# import generated.messages_pb2 as message_pb2
from sw.IO.actionneurs import IO_Manager


MAX_SPEED = 300
V_THETA = 2

FRAME_W = 1
FRAME_R = 0

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
    "X": 4, #axe 1
    "X_sens" : -1, #facteur
    "X_offset": 0.15,
    "X_dead_zone": 0.1,
    "Y": 3, #axe 0
    "Y_sens" : -1,
    "Y_offset" : -0.6,
    "Y_dead_zone": 0.1,
    "THETA": 0, #axe 3
    "THETA_sens" : -1,
    "THETA_offset" : 0.1, #à régler
    "THETA_dead_zone": 0.1, # à régler
    "theta_supra_luminique":11, #bouton
    "vitesse_supra_luminique": 12, #bouton
    "glisse" : 1, #hat (fleche haut et bas)
    "selectGlisse" : 0, #hat (fleche gauche et droite) 
    "bras" : 6, #bouton gachette gauche
    "verrou" : 7, #bouton gachette droite
    "grabHaut" : 4, #bouton bumper gauche
    "grabBas" : 5, #bouton bumper droit
    "action_1" : 0, #bouton bumper gauche
    "action_2" : 2, #bouton bumper gauche

}
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
        self.last_time = 0
        self.cons_timer_on = False
        ecal_core.initialize([], "Joystick")
        time.sleep(1) # on laisse ecal se reveiller

        self.speed_publisher = ProtoPublisher("speed_cons", common_pb.Speed)
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
        
        #print(self.message)
        cons_at_0 = abs(self.message.vx) < 0.1 and abs(self.message.vy) < 0.1 and abs(self.message.vtheta) <= 0.1
        if cons_at_0:
            if not self.cons_timer_on:
                self.last_time = time.time()
                self.cons_timer_on = True
            if self.cons_timer_on :
                timeout = abs(time.time()- self.last_time) > TIMEOUT_COMMAND
        else:
            timeout = False
            self.cons_timer_on = False

        if not timeout:
            self.speed_publisher.send(self.message)

if __name__ == '__main__':
    pygame.init()
    pygame.joystick.init()

    joysticks_ecal = JoystickEcal()        
    joysticks_ecal.open()
    joysticks_ecal.set_conf(BATTLETRON)
    #tree = py_trees.trees.BehaviourTree(joystick_bt(joysticks_ecal.IO_manager))
    #tree.setup(timeout=15)
    #tick = 0
    while True :
        for event in pygame.event.get():
            # print(event)
            joysticks_ecal.update_value()
            # print(f'Glissière :{joysticks_ecal.glisseMode}\n')
            # print(joysticks_ecal)
        joysticks_ecal.publish_command()
        #print(f"Tick {tick} \n")
        #tree.tick()
        #tick = tick + 1
        time.sleep(0.1)
    

