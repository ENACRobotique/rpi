#!/usr/bin/python3
from serial import Serial
from enum import Enum
from typing import Callable
from threading import Thread
import time


POTAR_RESOLUTION = 50

class Button(Enum):
    OK = 0
    RET = 1
    COLOR = 2
    TIRETTE = 3
    POTAR = 4       # not a button !

class Event(Enum):
    PRESSED = 1
    RELEASED = 2

class LCD(Thread):
    def __init__(self, port, event_cb: Callable[[Button, Event], None] = None) -> None:
        Thread.__init__(self)
        self._ser = Serial(port, 115200, timeout=0.5)
        self._state = {
            Button.OK: False,
            Button.RET: False,
            Button.COLOR: False,
            Button.TIRETTE: False,
            Button.POTAR: 0
        }
        self.event_cb = event_cb
        self.running = True
        self.start()
    
    def get_state(self, btn: Button):
        return self._state[btn]
    
    def display(self, line1: str, line2: str,
                red: bool, green: bool, blue: bool,
                buzz: int = ord('0')):
        line1 = line1.ljust(16)
        line2 = line2.ljust(16)
        r = b'1' if red else b'0'
        g = b'1' if green else b'0'
        b = b'1' if blue else b'0'

        if isinstance(buzz, bytes) or isinstance(buzz, str):
            buzz = ord(buzz)
        if buzz < ord('A') or buzz > ord('A')+7*3:
            buzz = ord('0')
        buzz_b = chr(buzz).encode()

        payload = line1.encode() + line2.encode() + r + g + b + buzz_b + b'\0'
        self._ser.write(payload)

    def stop(self):
        self.running = False
        self.join()

    def run(self):
        while self.running:
            line = self._ser.readline().strip()
            
            #timeout
            if line == b'':
                continue
            *buttons, potar = line.split()
            try:
                potar = int(potar)
            except ValueError as e:
                print(e)
                potar = self._state[Button.POTAR]

            if len(buttons) > 4:
                print(f"Too many elements: {line}")
                continue

            # buttons
            for i, val in enumerate(buttons):
                btn = Button(i)
                if val in b'PC':
                    self._state[btn] = True
                elif val in b'RO':
                    self._state[btn] = False
                
                if val in b'PR' and self.event_cb is not None:
                    self.event_cb(btn, val)
            
            # potar
            diff = abs(potar - self._state[Button.POTAR])
            if diff >= POTAR_RESOLUTION:
                self._state[Button.POTAR] = potar - (potar % POTAR_RESOLUTION)
                self.event_cb(Button.POTAR, self._state[Button.POTAR])
                


if __name__ == '__main__':
    def cb(btn: Button, val):
        notes = ['C', 'E', 'G', ord('C')+7, '0']
        leds = {
            Button.OK: (False, True, False),
            Button.COLOR: (False, False, True),
            Button.RET: (True, False, False),
            Button.TIRETTE: (True, False, True),
            Button.POTAR: (False, True, True),
        }
        buzz = notes[btn.value]
        valstr = val.decode() if isinstance(val, bytes) else str(val)
        lcd.display(btn.name, valstr, *leds[btn], buzz)
        print(btn, val)
    
    lcd = LCD('/dev/ttyUSB1', cb)
    lcd.display("Bonjour", "les gens", False, True, False)
    while True:
        # lcd.display("Bonjour", "les gens", False, True, False)
        # time.sleep(2)
        # lcd.display("Comment", "allez-vous ?", False, False, True)
        time.sleep(2)
