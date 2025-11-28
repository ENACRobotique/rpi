#!/usr/bin/python3
from lcd_driver import *
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Publisher as ProtoPublisher
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.common.core import ReceiveCallbackData

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
import generated.robot_state_pb2 as rpb


class ECAL_LCD:
    def __init__(self, port) -> None:
        if not ecal_core.is_initialized():
            ecal_core.initialize("eCAL_LCD_driver")
        self.state_pub = ProtoPublisher(rpb.LCDState, "LCDState")
        self.event_pub = ProtoPublisher(rpb.LCDEvent, "LCDEvent")
        self.display_sub = ProtoSubscriber(rpb.LCDOut, "LCDOut")
        self.display_sub.set_receive_callback(self.display_lcd)
        self.lcd = LCD(port, self.lcd_event_cb)

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        self.display_sub.remove_receive_callback()

    
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
    
    def display_lcd(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[rpb.LCDOut]):
        msg = data.message
        self.lcd.display(msg.line1, msg.line2, msg.red, msg.green, msg.blue, msg.buzzer)


if __name__ == '__main__':
    with ECAL_LCD('/dev/robot_lcd') as ecal_lcd :
        while ecal_core.ok():
            time.sleep(1)
            ecal_lcd.send_state()
        ecal_core.finalize()
    
