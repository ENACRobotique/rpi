#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <unistd.h>
#include <inttypes.h>
#include <iostream>
#include <thread>
#include <stdio.h>
#include <fcntl.h>
#include "actionneurs.pb.h"
#include "math.h"
#include "STS3032.h"
#include "serial.h"
#include "Dynamixel.h"

using SS = enac::SmartServo;
STS3032* p_sts;
Dynamixel* p_dynamixel;

void handleServo(const SS& msg, SmartServo* p_servo);
void ssCb(const SS& msg);

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
  sts.setSerialBaudrate(125000);
  p_sts = &sts;

  // eCAL 
  eCAL::Initialize(argc, argv, "smart_servo_driver");
  eCAL::protobuf::CSubscriber<SS> subscriber("smart_servo");
  // eCAL::CServiceServer server("smartServo Server");
  subscriber.AddReceiveCallback(std::bind(&ssCb, std::placeholders::_2));

  while (eCAL::Ok()){
    usleep(1);}

  eCAL::Finalize();
}

void ssCb(const SS& msg){
  SmartServo* p_servo;
  if (msg.type() == SS::ServoType::SmartServo_ServoType_STS)
    {p_servo = p_sts;}

  else if (msg.type() == SS::ServoType::SmartServo_ServoType_AX12)
    {p_servo = p_dynamixel;}

  handleServo(msg, p_servo);
  return;
}

void handleServo(const SS& msg, SmartServo* p_servo)
{
    switch (msg.command())
    {
        case SS::CommandType::SmartServo_CommandType_SET_ID:
            p_servo->setID((uint8_t)msg.id(), (uint8_t)msg.newid());
            break;
        
        case SS::CommandType::SmartServo_CommandType_PING:
            p_servo->ping((uint8_t)msg.id());
            break;

        case SS::CommandType::SmartServo_CommandType_SET_BAUDRATE:
            p_servo->setBaudrate((uint8_t)msg.id(), (uint32_t)msg.speed());
            break;

        case SS::CommandType::SmartServo_CommandType_MOVE:
            p_servo->move((uint8_t)msg.id(), (uint16_t)msg.position());
            break;

        case SS::CommandType::SmartServo_CommandType_MOVE_SPEED:
            p_servo->moveSpeed((uint8_t)msg.id(), (uint16_t)msg.position(), (uint16_t)msg.speed());
            break;

        case SS::CommandType::SmartServo_CommandType_SET_ENDLESS:
            p_servo->setEndless((uint8_t)msg.id(), msg.endless_status());
            break;

        case SS::CommandType::SmartServo_CommandType_TURN:
            // je sais pas si le cast marchera
            p_servo->turn((uint8_t)msg.id(), (SmartServo::RotationDirection)msg.direction(), (uint16_t)msg.speed());
            break;

        case SS::CommandType::SmartServo_CommandType_SET_TORQUE:
            p_servo->setTorque((uint8_t)msg.id(), (uint16_t)msg.torque());
            break;

        case SS::CommandType::SmartServo_CommandType_TORQUE_ENABLE:
            p_servo->torqueEnable((uint8_t)msg.id(), msg.enable_torque());
            break;

        case SS::CommandType::SmartServo_CommandType_SET_LIMITS:
            p_servo->setLimits((uint8_t)msg.id(), (uint16_t)msg.min_angle(), (uint16_t)msg.max_angle());
            break;

        default:
            std::cerr << " Unknown command : " << msg.command() << std::endl;
            break;
}
}
