#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <unistd.h>
#include <inttypes.h>
#include <iostream>
#include <thread>
#include <stdio.h>
#include <fcntl.h>
#include "lidar_data.pb.h"
#include "math.h"
#include "STS3032.h"
#include "serial.h"


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

  while(true) {
    sts.ping(1);
    usleep(500000);
  }

  
  // Initialize eCAL and create a protobuf publisher
  // eCAL::Initialize(argc, argv, "ld06_driver");
  // eCAL::protobuf::CPublisher<enac::Lidar> publisher("lidar_data");


  // LD06 ld06([&publisher](std::array<Point,LD06_NB_POINTS> points, size_t len){
  //   enac::Lidar lidar_msg;
  //   for (int i=len-1; i>=0; i--){
  //       lidar_msg.add_angles((360-points[i].angle) * M_PI / 180.0);
  //       lidar_msg.add_distances(points[i].distance);
  //       lidar_msg.add_quality(points[i].intensity);
  //   }
  //   // Send the message
  //   publisher.Send(lidar_msg);

  // });


  // Infinite loop (using eCAL::Ok() will enable us to gracefully shutdown the
  // Process from another application)
  // while (eCAL::Ok()){
  //   uint8_t cs[SERIAL_BUF_LEN];

  //   int num_bytes = read(serial_port, cs, SERIAL_BUF_LEN);
  //   ld06.feed(cs, num_bytes);
  // }

  // finalize eCAL API
  // eCAL::Finalize();
}
