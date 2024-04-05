import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
import sys,time
import generated.robot_state_pb2 as robot_pb
import robot

if __name__ == "__main__" :
    
    print("test runnning")
    
    robot = robot.Robot()
    time.sleep(0.1)

    robot.initNav()
    robot.pathFinder("secureB","secureJ")
    
    # faire le reset 1 fois suffit pas alors bon ...
    for i in range(10000):
        robot.resetPosFromNav("secureB") # ou pas forcément
    time.sleep(0.1)
    
    while ecal_core.ok():
        robot.followPath()
        if robot.isNavDestReached():
            break
        time.sleep(0.1)


