#!/usr/bin/python3
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber
import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import generated.robot_state_pb2 as rpb
from abc import ABC, abstractmethod
from threading import Thread
from enum import Enum


class Page(ABC):
    def __init__(self,title, parent) -> None:
        self.title = title
        self.parent = parent
        self.need_refresh = False

    @abstractmethod
    def handle_event(self, event: rpb.LCDEvent):
        print(event)
    
    @abstractmethod
    def display(self) -> tuple[str, str]:
        ...
    
    def update(self) -> None:
        ...
    
    def invalidate(self):
        self.need_refresh = True


class Menu(Page):
    def __init__(self, title, parent) -> None:
        super().__init__(title, parent)
        self.subpages: list[Page] = []
        self.current_index = 0
        
    def add_subpages(self, *pages: Page):
        for page in pages:
            self.subpages.append(page)
    
    def handle_event(self, event: rpb.LCDEvent):
        if event.button == rpb.LCDEvent.Button.OK and event.value == 1:
            if len(self.subpages):
                return self.subpages[self.current_index]
        elif event.button == rpb.LCDEvent.Button.RET and event.value == 1:
            if self.parent is not None:
                return self.parent
        elif event.button == rpb.LCDEvent.Button.POTAR:
            self.current_index = int(event.value * len(self.subpages) / 1050)
    
    def display(self):
        if self.current_index < len(self.subpages):
            return self.title, self.subpages[self.current_index].title
        else:
            return self.title, "-----"

class Choice(Page):
    def __init__(self, title, parent, choices: list[str|Enum], ok_cb) -> None:
        super().__init__(title, parent)
        self.choices = choices
        self.ok_cb = ok_cb
        self.current_index = 0
        self.commited_index = -1
    
    def handle_event(self, event: rpb.LCDEvent):
        if event.button == rpb.LCDEvent.Button.OK and event.value == 1:
            if self.ok_cb is not None:
                self.ok_cb(self.choices[self.current_index])
            self.commited_index = self.current_index
        elif event.button == rpb.LCDEvent.Button.RET and event.value == 1:
            if self.parent is not None:
                return self.parent
        elif event.button == rpb.LCDEvent.Button.POTAR:
            self.current_index = int(event.value * len(self.choices) / 1050)
    
    def display(self):
        if self.current_index > len(self.choices):
            return self.title, "----------------"
        if self.current_index == self.commited_index:
            title = self.title
        else:
            title = self.title.ljust(15)+'*'
        choice = self.choices[self.current_index]
        if isinstance(choice, Enum):
            choice = choice.name
        return title, choice
        

class Number(Page):
    def __init__(self, title, parent, min, max, ok_cb) -> None:
        super().__init__(title, parent)
        self.min = min
        self.delta = max - min
        self.value = 0
        self.commited_value = 0
        self.ok_cb = ok_cb
    
    def handle_event(self, event: rpb.LCDEvent):
        if event.button == rpb.LCDEvent.Button.OK and event.value == 1:
            if self.ok_cb is not None:
                self.ok_cb(self.value)
            self.commited_value = self.value
        elif event.button == rpb.LCDEvent.Button.RET and event.value == 1:
            if self.parent is not None:
                return self.parent
        elif event.button == rpb.LCDEvent.Button.POTAR:
            self.value = self.min + self.delta * event.value / 1000
    
    def display(self):
        if self.value == self.commited_value:
            title = self.title
        else:
            title = self.title.ljust(15)+'*'
        return title, str(self.value)

class Text(Page):
    def __init__(self, title, parent, *lines) -> None:
        super().__init__(title, parent)
        if len(lines) > 1:
            self.line1 = lines[0]
            self.line2 = lines[1]
        else:
            self.line1 = title
            self.line2 = lines[0]
        self.need_refresh = True
    
    def handle_event(self, event: rpb.LCDEvent):
        if event.button == rpb.LCDEvent.Button.OK and event.value == 1:
            ...
        elif event.button == rpb.LCDEvent.Button.RET and event.value == 1:
            if self.parent is not None:
                return self.parent
        elif event.button == rpb.LCDEvent.Button.POTAR:
            ...
    
    def display(self):
        self.need_refresh = False
        return self.line1, self.line2

    def set_text(self, *lines):
        if len(lines) > 1:
            self.line1 = lines[0]
            self.line2 = lines[1]
        else:
            self.line2 = lines[0]
        self.invalidate()


class LCDClient(Thread):
    def __init__(self, page_init: Page, on_event, on_state) -> None:
        Thread.__init__(self)
        self.state_sub = ProtoSubscriber("LCDState", rpb.LCDState)
        self.event_sub = ProtoSubscriber("LCDEvent", rpb.LCDEvent)
        self.display_pub = ProtoPublisher("LCDOut", rpb.LCDOut)
        self.current_page = page_init
        self.event_sub.set_callback(self.event_cb)
        self.state_sub.set_callback(self.state_cb)
        self.on_event = on_event
        self.on_state = on_state
        self.red = False
        self.green = False
        self.blue = False
        self.buzz = ord('0')

    def set_page(self, page: Page):
        self.current_page = page
    
    def display(self):
        line1, line2 = self.current_page.display()
        msg = rpb.LCDOut(line1=line1, line2=line2, red=self.red, green=self.green, blue=self.blue, buzzer=self.buzz)
        self.display_pub.send(msg)
    
    def event_cb(self, topic_name, msg, timestamp):
        self.on_event(msg)
        # if msg.button == rpb.LCDEvent.Button.COLOR and msg.value == 1:
        #     print("color!!!")
        # elif msg.button == rpb.LCDEvent.Button.TIRETTE and msg.value == 0:
        #     print("tirette!!!")
        ret = self.current_page.handle_event(msg)
        if ret is not None:
            self.current_page = ret
        self.display()
    
    def state_cb(self, topic_name, msg, timestamp):
        if self.on_state is not None:
            self.on_state(msg)
    
    def run(self):
        while True:
            self.current_page.update()  # ????? a garder ou pas ?
            if self.current_page.need_refresh:
                self.display()
            time.sleep(0.2)


if __name__ == '__main__':
    def color_cb(color):
        print(color)
    ecal_core.initialize(sys.argv, "LCD_client")
    m=Menu("Robot", None)
    color_choices = Choice("Couleur", m, ["Bleu", "Jaune"], color_cb)
    tt = Text("Simple Text", m, "plop")
    strat_choices = Choice("Strat", m, ["Basique", "audacieuse"], lambda x: print(x))
    detect_range = Number("Dist detection", m, 20, 150, 5)
    tt = Text("Simple Text", m, "plop")
    m.add_subpages(color_choices, strat_choices, detect_range)
    l = LCDClient(m)
    
    while True:
        l.display()
        time.sleep(10)

# position courante
