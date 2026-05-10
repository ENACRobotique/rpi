from UCD import xbee
from generated.robot_state_pb2 import Aruco_UCD
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Publisher as ProtoPublisher
import argparse

class Lecteur_aruco_ucd:

    def __init__(self) -> None:
        self.msg = Aruco_UCD()

    def cb(self, _sender, data):
        msg_resp = Aruco_UCD()
        msg_resp.ParseFromString(data)
        if msg_resp.frame_id == self.msg.frame_id:
            self.msg.ArucoId.extend(msg_resp.ArucoId)
            self.msg.pos.extend(msg_resp.pos)
        else:     
            aruco_UCD_pub.send(self.msg)
            self.msg = Aruco_UCD(frame_id=msg_resp.frame_id, pos= msg_resp.pos, ArucoId=msg_resp.ArucoId)
  


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument('name', help='xbee path (eg : /dev/ttyUSB0)')
    parser.add_argument('id', help='int')
    args = parser.parse_args()
    ecal_core.initialize("aruco_UCD")
    aruco_UCD_pub = ProtoPublisher(Aruco_UCD, "aruco_UCD")

    lecteur = Lecteur_aruco_ucd()
    x=xbee.Xbee(lecteur.cb, args.id, args.name)
    x.start()