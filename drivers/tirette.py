#!/usr/bin/env python3
import gpiod
from gpiod.line import Direction, Bias, Edge, Value
import datetime
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Publisher as ProtoPublisher
import robot_state_pb2  as rpb


LINE = 25
TIMEOUT = 3

LINE_COLOR = 16


if __name__ == "__main__":
    tirette_request = gpiod.request_lines(
    "/dev/gpiochip4",
    consumer="blink-example",
    config={
        LINE: gpiod.LineSettings(
            direction=Direction.INPUT,
            bias=Bias.PULL_UP,
            edge_detection=Edge.BOTH,
            debounce_period=datetime.timedelta(seconds=0.1)
        ),
        LINE_COLOR: gpiod.LineSettings(
            direction=Direction.INPUT,
            bias=Bias.PULL_UP,
            edge_detection=Edge.BOTH,
            debounce_period=datetime.timedelta(seconds=0.1)
        )
        },
    )

    if not ecal_core.is_initialized():
            ecal_core.initialize("Tirette")
    tirette_pub = ProtoPublisher(rpb.Tirette, "tirette")
    color_pub = ProtoPublisher(rpb.Side,"color")
    
    while ecal_core.ok():
        if tirette_request.wait_edge_events(timeout=TIMEOUT):
            for event in tirette_request.read_edge_events():
                pass
                #print(event)
        pass
        val = tirette_request.get_value(LINE)
        tirette_state = rpb.Tirette.IN if val == Value.INACTIVE else rpb.Tirette.OUT
        valColor = tirette_request.get_value(LINE_COLOR)
        color_state = rpb.Side.BLUE if valColor == Value.INACTIVE else rpb.Side.YELLOW

        msg = rpb.Tirette(tirette_state=tirette_state)
        tirette_pub.send(msg)
        print(val, tirette_state)

        msg_c = rpb.Side(color = color_state)
        color_pub.send(msg_c)

