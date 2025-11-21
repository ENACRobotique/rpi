#!/usr/bin/python3
from serial import Serial
from time import sleep
import sys
from robot import Robot, Pos, Frame
from math import cos, sin, pi

THETA_PINCE = -pi/3

PLANTE = 870
POT = 920
OPEN = 1100
WIDE_OPEN = 1600

BAS = 230
HAUT = 550


POS_POT = Pos(350, 350, 3*pi/4 - THETA_PINCE)
POS_POT2 = Pos(150, 0, 0)
POS_JARDINIERE = Pos(200, 0, 0)


frame_pince = Pos(0, 0, THETA_PINCE)


class PlanteTest:
    def __init__(self):
        self.ser = Serial("/dev/ttyACM1", 115200)

    def levage(self, position):
        self.ser.write(f"dyn 5 {position} 200\r\n".encode())
        sleep(1)

    def pince(self, position):
        self.ser.write(f"servo 1 {position}\r\n".encode())
        sleep(1)


def wait_target(r: Robot):
    while not r.hasReachedTarget():
        sleep(0.5)

if __name__ == "__main__":
    with Robot() as r:
        sleep(1)
        r.resetPos(Pos(120, 250, 0))
        sleep(0.1)
        r.resetPos(Pos(120, 250, 0))
        sleep(0.1)
        r.resetPos(Pos(120, 250, 0))
        t = PlanteTest()

        sleep(2)

        t.pince(OPEN)
        t.levage(BAS)


        r.setTargetPos(Pos(700, 700, -THETA_PINCE))
        wait_target(r)
        
        # avancer plante
        r.move(100, THETA_PINCE)
        wait_target(r)

        t.pince(PLANTE)

        # reculer
        r.move(-100, THETA_PINCE)
        wait_target(r)

        t.levage(HAUT)

        r.setTargetPos(POS_POT)
        wait_target(r)
        
        # avancer pot
        r.move(70, THETA_PINCE)
        wait_target(r)

        t.pince(OPEN)

        r.move(-80, THETA_PINCE)
        wait_target(r)

        t.levage(BAS)

        r.move(160, THETA_PINCE)
        wait_target(r)

        t.pince(POT)
        t.levage(HAUT)

        r.setTargetPos(Pos(150, 600, pi-THETA_PINCE))
        wait_target(r)

        t.pince(OPEN)

        r.move(-100, THETA_PINCE)

        exit(0)
