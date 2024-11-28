#!/usr/bin/python3
from lcd_driver import *
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
import generated.robot_state_pb2 as rpb


class ECAL_LCD:
    def __init__(self, port) -> None:
        ecal_core.initialize(sys.argv, "eCAL_LCD_driver")
        self.state_pub = ProtoPublisher("LCDState", rpb.LCDState)
        self.event_pub = ProtoPublisher("LCDEvent", rpb.LCDEvent)
        self.display_sub = ProtoSubscriber("LCDOut", rpb.LCDOut)
        self.display_sub.set_callback(self.display_lcd)
        self.lcd = LCD(port, self.lcd_event_cb)
    
    def lcd_event_cb(self, btn: Button, val):
        if isinstance(val, bytes):
            val = 1 if val in b'PC' else 0
        msg = rpb.LCDEvent(button=btn.value, value=val)
        self.event_pub.send(msg)

    def send_state(self):
        msg = rpb.LCDState()
        msg.ok = int(self.lcd.get_state(Button.OK))
        msg.ret = int(self.lcd.get_state(Button.RET))
        msg.color = int(self.lcd.get_state(Button.COLOR))
        msg.tirette = int(self.lcd.get_state(Button.TIRETTE))
        msg.potar = self.lcd.get_state(Button.POTAR)
        self.state_pub.send(msg)
    
    def display_lcd(self, topic_name, msg, timestamp):
        self.lcd.display(msg.line1, msg.line2, msg.red, msg.green, msg.blue, msg.buzzer)


if __name__ == '__main__':
    ecal_lcd = ECAL_LCD('/dev/robot_lcd')

    while ecal_core.ok():
        time.sleep(1)
        ecal_lcd.send_state()
    
