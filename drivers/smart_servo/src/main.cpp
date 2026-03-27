#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <ecal/service/server.h>
#include <unistd.h>
#include <inttypes.h>
#include <iostream>
#include <thread>
#include <stdio.h>
#include <fcntl.h>
#include <stdarg.h>
#include "actionneurs.pb.h"
#include "math.h"
#include "serial.h"
#include "smart_servo.h"

using SAPRecord = enac::SAPRecord;

SmartServo* p_sap;


void log_msg(const char *fmt, ...) {
  char print_buffer[200] = {0};
  va_list args;
  va_start(args, fmt);
  vsnprintf(print_buffer, 200, fmt, args);
  va_end(args);
  eCAL::Logging::Log(eCAL::Logging::eLogLevel::log_level_info, print_buffer);
}

int read_reg(const eCAL::SServiceMethodInformation& method_info_,
         const std::string& request_,
         std::string& response_) {

  SAPRecord msg;
  msg.ParseFromString(request_);
  SmartServo::record_t record = {
    .id = (uint8_t) msg.id(),
    .reg = (uint8_t) msg.reg(),
    .len = (uint8_t) msg.len(),
  };
  SmartServo::Status status = p_sap->read(&record);
  msg.set_status(status);
  if(status == SmartServo::Status::OK) {
    msg.set_data(record.data, record.len);
  }
  msg.SerializeToString(&response_);

  return 0;
}

int write_reg(const eCAL::SServiceMethodInformation& method_info_,
         const std::string& request_,
         std::string& response_) {

  SAPRecord msg;
  msg.ParseFromString(request_);
  SmartServo::record_t record = {
    .id = (uint8_t) msg.id(),
    .reg = (uint8_t) msg.reg(),
    .len = (uint8_t) msg.data().size(),
  };
  memcpy(record.data, msg.data().data(), msg.data().size());
  SmartServo::Status status = p_sap->write(&record);
  msg.set_status(status);

  msg.SerializeToString(&response_);

  return 0;
}

int ping(const eCAL::SServiceMethodInformation& method_info_,
         const std::string& request_,
         std::string& response_) {

  SAPRecord msg;
  msg.ParseFromString(request_);
  
  SmartServo::Status status = p_sap->ping(msg.id());
  msg.set_status(status);

  if(status == SmartServo::Status::OK) {
    msg.set_data("OK");
  } else {
    msg.set_data("BAD");
  }

  msg.SerializeToString(&response_);

  return 0;
}

int main(int argc, char** argv){

  if(argc < 2) {
    std::cout << "Please specify serial port" << std::endl;
    return -1;
  }

  int baudrate = 500000;
  if(argc == 3) {
    baudrate = atoi(argv[2]);
    if(baudrate == 0) {
      baudrate = 500000;
    }
  }
  
  int serial_port = open(argv[1], O_RDWR);

  if(init_serial(serial_port, baudrate)) {
    std::cout << "Error configuring serial port!" << std::endl;
  }
  
  initDriver(19);
  
  SmartServo sap_controller(serial_port);


  sap_controller.setSerialBaudrate(baudrate);
  p_sap = &sap_controller;


  eCAL::Initialize("smart_servo_driver");


  eCAL::CServiceServer server("actuators");
  
  server.SetMethodCallback({"read_reg", "SAPRecord", "SAPRecord"}, read_reg);
  
  server.SetMethodCallback({"write_reg", "SAPRecord", "SAPRecord"}, write_reg);

  server.SetMethodCallback({"ping", "SAPRecord", "SAPRecord"}, ping);


  
  while (eCAL::Ok()){
    usleep(1000);
  }

  eCAL::Finalize();
}
