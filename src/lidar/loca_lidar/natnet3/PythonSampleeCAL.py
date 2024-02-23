from NatNetClient import NatNetClient
from pyquaternion import Quaternion
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
import robot_state_pb2 as robot_pb


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
publishers = {}

def send_rigibody(id, x, y, theta):
  if id not in publishers:
    publishers[id] = ProtoPublisher(f"{id}_pos", robot_pb.Position)
  print(x,y, theta)
  msg = robot_pb.Position()
  msg.x = float(x)
  msg.y = float(y)
  msg.theta = float(theta)
  publishers[id].send(msg, 
                      time=ecal_core.getmicroseconds()[1])

def receiveRigidBodyFrame( rblist, stamp ):
  print(stamp)
  for rb in rblist:
    id, pos, rot, valid = rb
    if valid:
      x,y,z = pos
      q=Quaternion(rot)
      theta = -q.yaw_pitch_roll[2]
      send_rigibody(id, x, y, theta)
      print(f"{id}: x={x:0.2f}, y={y:0.2f}   {theta}")
    else:
      print(f"{id} invalid")

ecal_core.initialize([], "Python_sample_natnet3")

# This will create a new NatNet client
streamingClient = NatNetClient(
server="192.168.1.231",
rigidBodyListListener=receiveRigidBodyFrame,
verbose=False,
)

streamingClient.run()
