#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
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

using SS = enac::SmartServo;
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

void handleServo(const SS& msg, SmartServo* p_servo);

int main(int argc, char** argv){

  if(argc < 2) {
    std::cout << "Please specify serial port" << std::endl;
    return -1;
  }
  
  int serial_port = open(argv[1], O_RDWR);

  if(init_serial(serial_port, B230400)) {
    std::cout << "Error configuring serial port!" << std::endl;
  }
  
  initDriver(0);

  STS3032 sts(serial_port);
  sts.init();
  sts.setSerialBaudrate(B500000);
  p_sts = &sts;

  eCAL::Initialize(argc, argv, "smart_servo_driver");
  eCAL::protobuf::CSubscriber<SS> subscriber("smart_servo");

  subscriber.AddReceiveCallback([=](const char *topic_name_, const SS &msg, long long time_, long long clock_, long long id_) {
    if (msg.type() == SS::ServoType::SmartServo_ServoType_STS) {
      handleServo(msg, p_sts);
    }
    else if (msg.type() == SS::ServoType::SmartServo_ServoType_AX12) {
      handleServo(msg, p_dynamixel);
    }
  });
  
  while (eCAL::Ok()){
    usleep(1000);
  }

  eCAL::Finalize();
}


void handleServo(const SS& msg, SmartServo* p_servo)
{
    switch (msg.command())
    {
        case SS::CommandType::SmartServo_CommandType_SET_ID:
            log_msg("[%d] set id to %d", msg.id(), msg.newid());
            p_servo->setID(msg.id(), msg.newid());
            break;
        
        case SS::CommandType::SmartServo_CommandType_PING:
            log_msg("[%d] ping", msg.id());
            p_servo->ping(msg.id());
            break;
        case SS::CommandType::SmartServo_CommandType_READ_POS:
          {
            int pos = p_servo->readPosition(msg.id());
            log_msg("[%d] Read pos: %d", msg.id(), pos);
          }
            break;

        case SS::CommandType::SmartServo_CommandType_SET_BAUDRATE:
            p_servo->setBaudrate(msg.id(), msg.speed());
            break;

        case SS::CommandType::SmartServo_CommandType_MOVE:
            p_servo->move(msg.id(), msg.position());
            break;

        case SS::CommandType::SmartServo_CommandType_MOVE_SPEED:
            p_servo->moveSpeed(msg.id(), msg.position(), msg.speed());
            break;

        case SS::CommandType::SmartServo_CommandType_SET_ENDLESS:
            p_servo->setEndless(msg.id(), msg.endless_status());
            break;

        case SS::CommandType::SmartServo_CommandType_TURN:
            p_servo->turn(msg.id(), (SmartServo::RotationDirection)msg.direction(), msg.speed());
            break;

        case SS::CommandType::SmartServo_CommandType_SET_TORQUE:
            p_servo->setTorque(msg.id(), msg.torque());
            break;

        case SS::CommandType::SmartServo_CommandType_TORQUE_ENABLE:
            p_servo->torqueEnable(msg.id(), msg.enable_torque());
            break;

        case SS::CommandType::SmartServo_CommandType_SET_LIMITS:
            log_msg("[%d] Set limits: %d-%d", msg.id(), msg.min_angle(), msg.max_angle());
            p_servo->setLimits(msg.id(), msg.min_angle(), msg.max_angle());
            break;

          case SS::CommandType::SmartServo_CommandType_SET_MULTITURN:
            log_msg("[%d] Set multiturn factor: %d", msg.id(), msg.multiturn_factor());
            if(msg.unlock_eeprom()) {
              p_servo->lock_eprom(msg.id(), false);
            }
            p_servo->setMultiturn(msg.id(), msg.multiturn_factor());
            if(msg.unlock_eeprom()) {
              p_servo->lock_eprom(msg.id(), true);
            }
            break;

        default:
            std::cerr << " Unknown command : " << msg.command() << std::endl;
            break;
    }
}
