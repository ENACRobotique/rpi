#!/usr/bin/env python3
import time
import ecal.nanobind_core as ecal_core
from generated.actionneurs_pb2 import SAPRecord


RESP_TIMEOUT = 100


class SAPInstructions:
  def __init__(self) -> None:
    if not ecal_core.is_initialized():
      ecal_core.initialize("smart servo test publisher")
    self.client = ecal_core.ServiceClient("actuators")
    while not self.client.is_connected():
      print("Waiting for SAP service ...")
      time.sleep(1.0)

  def ping(self, id):
    rec = SAPRecord(id=id)
    response = self.client.call_with_response("ping", rec.SerializeToString(), RESP_TIMEOUT)
    if response is None:
      print("no response... service offline ?")
      return False
    msg_resp = SAPRecord()
    msg_resp.ParseFromString(response[0].response)
    if msg_resp.id == id and msg_resp.data == b'OK':
      return True
    return False
  
  def write(self, id,  reg, data):
    rec = SAPRecord(id=id, reg=reg, len=len(data), data=data)
    msg_bin = rec.SerializeToString()
    self.client.call_with_callback_async("write_reg", msg_bin, self.response_cb)

  def read(self, id, reg, len):
    rec = SAPRecord(id=id, reg=reg, len=len)
    response = self.client.call_with_response("read_reg", rec.SerializeToString(), RESP_TIMEOUT)
    if response is None:
      print("no response... service offline ?")
      return None
    msg_resp = SAPRecord()
    msg_resp.ParseFromString(response[0].response)
    if msg_resp.id == id and msg_resp.reg == reg and msg_resp.len == len and msg_resp.status == 0:
      return msg_resp.data
    return None
  
  def response_cb(self, *args):
    pass


