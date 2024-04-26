#!/usr/bin/env python3

import time
import RPi.GPIO as GPIO
import subprocess

BUTTON_GPIO = 25
if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    pressed = False
    while True:
        # button is pressed when pin is LOW
        if not GPIO.input(BUTTON_GPIO):
            if not pressed:
                print("Button pressed!")
                pressed = True
                #subprocess.Popen(["/home/pi/rpi2024/services/script_restart_wifi.sh"], start_new_session=True)
                subprocess.run(["/home/pi/rpi2024/services/script_restart_wifi.sh"])

        # button not pressed (or released)
        else:
            # print("Button released !")
            pressed = False
        time.sleep(0.1)
