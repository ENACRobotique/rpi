import gpiod
from gpiod.line import Direction, Bias, Edge
import datetime

LINE = 25
TIMEOUT = 3


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
        )
        },
    )

    while True:
        if tirette_request.wait_edge_events(timeout=TIMEOUT):
            for event in tirette_request.read_edge_events():
                print(event)
                # if event.event_type == event.Type.RISING_EDGE:
                #     print("rising edge")
                # elif event.event_type == event.Type.FALLING_EDGE:
                #     print("falling edge")
        val = tirette_request.get_value(LINE)
        print(val)
