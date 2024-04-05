import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
import sys,time
import generated.robot_state_pb2 as robot_pb
# import Roboto

if __name__ == "__main__" : 
    
    print("test runnning")
    
    robot = Roboto.Robot()
    robot.pathFinder("secureB","secureJ")
    robot.reset_pos_from_nav("secureB") # pas oublier de dire au robot qu'il est en secureB au départ et l'y placer IRL
    
    # while True :
    #     robot.followPath()
        
    while ecal_core.ok():
        time.sleep(0.1)


