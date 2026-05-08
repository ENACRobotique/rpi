#!/usr/bin/env python3
import time
import ecal.nanobind_core as ecal_core
from generated.actionneurs_pb2 import SAPRecord


RESP_TIMEOUT = 50
NB_RETRY = 10


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
    for _ in range(NB_RETRY):
      response = self.client.call_with_response("ping", rec.SerializeToString(), RESP_TIMEOUT)
      if response is not None:
        msg_resp = SAPRecord()
        msg_resp.ParseFromString(response[0].response)
        if msg_resp.id == id and msg_resp.data == b'OK':
          return True
        return False
    print(f"[SAP Ping]: failed to ping to {id}.")
    return False
  
  def write(self, id,  reg, data):
    rec = SAPRecord(id=id, reg=reg, len=len(data), data=data)
    msg_bin = rec.SerializeToString()
    for i in range(NB_RETRY):
      response = self.client.call_with_response("write_reg", msg_bin, RESP_TIMEOUT)
      if response is not None:
        return True
      print(f"[SAP Write] c'est raté bébé {id} ({i}/{NB_RETRY})")
    print(f"[SAP Write]: failed to write to {id}.")
    return False

  def read(self, id, reg, len):
    rec = SAPRecord(id=id, reg=reg, len=len)
    for i in range(NB_RETRY):
      response = self.client.call_with_response("read_reg", rec.SerializeToString(), RESP_TIMEOUT)
      if response is not None:
        msg_resp = SAPRecord()
        msg_resp.ParseFromString(response[0].response)
        if msg_resp.id == id and msg_resp.reg == reg and msg_resp.len == len and msg_resp.status == 0:
          return msg_resp.data
        return None
      print(f"[SAP Read] c'est raté bébé {id} ({i}/{NB_RETRY})")
    print(f"[SAP Read] failed to read from {id}.")


