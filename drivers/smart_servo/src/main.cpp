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

using SS = enac::SmartServo;
STS3032* p_sts;

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
  printf("Smart Servo Drivers Terminated\n");
}

void ssCb(const SS& msg){
  if (msg.type() == SS::ServoType::SmartServo_ServoType_STS)
  {
    if (msg.command()== SS::CommandType::SmartServo_CommandType_PING)
    {
      p_sts->ping((uint8_t)msg.id());
      // printf("pinged sts %d\n",msg.id());
    }
  }
  return;
}