import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
import sys,time
import generated.robot_state_pb2 as robot_pb
import robot

if __name__ == "__main__" :
    
    print("test runnning")
    
    robot = robot.Robot()

    robot.initNav()
    robot.pathFinder("basJ","potSE")
    
    robot.resetPosFromNav("basJ") # ou pas forcément
    time.sleep(0.1)
    
    while ecal_core.ok():
        robot.followNav()
        if robot.isNavDestReached():
            break
        time.sleep(0.1)


