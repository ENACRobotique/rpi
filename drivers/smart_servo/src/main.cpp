#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <ecal/ecal_server.h>
#include <unistd.h>
#include <inttypes.h>
#include <iostream>
#include <thread>
#include <stdio.h>
#include <fcntl.h>
#include <stdarg.h>
#include "actionneurs.pb.h"
#include "math.h"
#include "STS3032.h"
#include "serial.h"
#include "Dynamixel.h"
#include "smart_servo.h"

using SS = enac::SmartServo;
using SAPRecord = enac::SAPRecord;
STS3032* p_sts;
Dynamixel* p_dynamixel;


void log_msg(const char *fmt, ...) {
  char print_buffer[200] = {0};
  va_list args;
  va_start(args, fmt);
  vsnprintf(print_buffer, 200, fmt, args);
  va_end(args);
  eCAL::Logging::Log(print_buffer);
}


int move(const std::string& method_,
         const std::string& req_type_,
         const std::string& resp_type_,
         const std::string& request_,
         std::string& response_) {

  SS msg;
  msg.ParseFromString(request_);

  SmartServo* p_servo;

  if (msg.type() == SS::ServoType::SmartServo_ServoType_STS) {
    p_servo = p_sts;
  }
  else if (msg.type() == SS::ServoType::SmartServo_ServoType_AX12) {
    p_servo = p_dynamixel;
  }


  switch (msg.command())
  {
      case SS::CommandType::SmartServo_CommandType_SET_ID:
          p_servo->setID(msg.id(), msg.new_id());
          response_ = request_;
          break;
      
      case SS::CommandType::SmartServo_CommandType_PING:
          p_servo->ping(msg.id());
          // TODO pong ?
          response_ = request_;
          break;
      case SS::CommandType::SmartServo_CommandType_READ_POS:
        {
          int pos = p_servo->readPosition(msg.id());
          msg.set_position(pos);
          msg.SerializeToString(&response_);
        }
          break;

      case SS::CommandType::SmartServo_CommandType_SET_BAUDRATE:
          p_servo->setBaudrate(msg.id(), msg.speed());
          response_ = request_;
          break;

      case SS::CommandType::SmartServo_CommandType_MOVE:
          p_servo->move(msg.id(), msg.position());
          response_ = request_;
          break;

      case SS::CommandType::SmartServo_CommandType_MOVE_SPEED:
          p_servo->moveSpeed(msg.id(), msg.position(), msg.speed());
          response_ = request_;
          break;

      case SS::CommandType::SmartServo_CommandType_SET_ENDLESS:
          p_servo->setEndless(msg.id(), msg.endless_status());
          response_ = request_;
          break;

      case SS::CommandType::SmartServo_CommandType_TURN:
          p_servo->turn(msg.id(), (SmartServo::RotationDirection)msg.direction(), msg.speed());
          response_ = request_;
          break;

      case SS::CommandType::SmartServo_CommandType_SET_TORQUE:
          p_servo->setTorque(msg.id(), msg.torque());
          response_ = request_;
          break;

      case SS::CommandType::SmartServo_CommandType_TORQUE_ENABLE:
          p_servo->torqueEnable(msg.id(), msg.enable_torque());
          response_ = request_;
          break;

      case SS::CommandType::SmartServo_CommandType_SET_LIMITS:
          p_servo->setLimits(msg.id(), msg.min_angle(), msg.max_angle());
          response_ = request_;
          break;

      case SS::CommandType::SmartServo_CommandType_SET_MULTITURN:
        if(msg.unlock_eeprom()) {
          p_servo->lock_eprom(msg.id(), false);
        }
        p_servo->setMultiturn(msg.id(), msg.multiturn_factor());
        if(msg.unlock_eeprom()) {
          p_servo->lock_eprom(msg.id(), true);
        }
        response_ = request_;
        break;

      case SS::CommandType::SmartServo_CommandType_IS_MOVING:
        if (msg.type() == SS::ServoType::SmartServo_ServoType_STS) {
          bool moving;
          SmartServo::Status status = ((STS3032*)p_servo)->isMoving(msg.id(), moving);
          if(status == SmartServo::Status::OK) {
            msg.set_moving(moving);
          } else {
            msg.set_moving(false);
          }
          msg.set_status((uint32_t)status);
          
          msg.SerializeToString(&response_);
        }
        break;
         
      default:
          std::cerr << " Unknown command : " << msg.command() << std::endl;
          break;
  }

  return 0;
}


int read_reg(const std::string& method_,
         const std::string& req_type_,
         const std::string& resp_type_,
         const std::string& request_,
         std::string& response_) {

  SAPRecord msg;
  msg.ParseFromString(request_);
  SmartServo::record_t record = {
    .id = (uint8_t) msg.id(),
    .reg = (uint8_t) msg.reg(),
    .len = (uint8_t) msg.len(),
  };
  SmartServo::Status status = p_sts->read(&record);
  if(status == SmartServo::Status::OK) {
    msg.set_data(record.data, record.len);
  }
  msg.SerializeToString(&response_);

  return 0;
}

int write_reg(const std::string& method_,
         const std::string& req_type_,
         const std::string& resp_type_,
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
  SmartServo::Status status = p_sts->write(&record);

  msg.SerializeToString(&response_);
  if(status == SmartServo::Status::OK) {
    return 0;
  }else{
    return 1;
  }
}


void handleServo(const SS& msg, SmartServo* p_servo);

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
  
  initDriver(0);

  STS3032 sts(serial_port);
  //sts.init();
  sts.setSerialBaudrate(baudrate);
  p_sts = &sts;

  Dynamixel dyn(serial_port);
  p_dynamixel = &dyn;
  

  eCAL::Initialize(argc, argv, "smart_servo_driver");


  eCAL::CServiceServer server("actuators");
  server.AddDescription("read_pos", "SS", "id", "SS", "position");
  server.AddMethodCallback("read_pos", "SS", "SS", move);
  
  server.AddDescription("read_reg", "SAPRecord", "", "SAPRecord", "");
  server.AddMethodCallback("read_reg", "SAPRecord", "SAPRecord", read_reg);
  
  server.AddDescription("write_reg", "SAPRecord", "", "SAPRecord", "");
  server.AddMethodCallback("write_reg", "SAPRecord", "SAPRecord", write_reg);
  server.Create("");


  
  while (eCAL::Ok()){
    usleep(1000);
  }

  eCAL::Finalize();
}
