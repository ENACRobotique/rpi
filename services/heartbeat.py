#!/bin/python3

from gpiozero import LED
from time import sleep


LED_GPIO = 19
PERIOD = 1

if __name__ == "__main__":
    led = LED(LED_GPIO)
    while True:
        led.on()
        sleep(PERIOD)
        led.off()
        sleep(PERIOD)
