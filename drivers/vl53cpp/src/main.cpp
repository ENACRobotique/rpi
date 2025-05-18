#include <stdio.h>
#include <array>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <string>
#include <gpiod.h>
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <lidar_data.pb.h>
extern "C"{
    #include <vl53l5cx_api.h>
}
#include "vl53.h"



#define GPIO_CHIP "gpiochip4"

#define RESULT_LENGTH (VL53L5CX_RESOLUTION_8X8*VL53L5CX_NB_TARGET_PER_ZONE)
// dans le sens des écritures I2C :
//16 haut gauche 
//17 bas droit
//18 bas gauche
//26 haut droit
int main(int argc, char* argv[]) {
    std::array sensors = {
	VL53L5CX(16, 0x30),//conserveDroit
        VL53L5CX(26, 0x32),//conserveGauche
        VL53L5CX(18, 0x34),//banderollle
        // VL53L5CX(16, 0x36),
    };

    std::map<VL53L5CX*,eCAL::protobuf::CPublisher<enac::Lidar>> publishers;
    // Initialize eCAL
    eCAL::Initialize(argc, argv, "vl53_driver");

    // create protobuf publishers
    for(int i=0; i<sensors.size(); i++){
        std::stringstream ss;
        ss << "vl53_" << i;
        publishers[&sensors[i]] = eCAL::protobuf::CPublisher<enac::Lidar>(ss.str());
    }

    
    struct gpiod_chip *chip = gpiod_chip_open_by_name(GPIO_CHIP);
    if (!chip) {
        std::cout << "Open chip failed\n";
		return -1;
	}


    //turn off all sensors
    for(auto& s:sensors) {
        s.turn_off(chip);
    }

    for (auto& s:sensors){
        s.init();
    }

    for (auto& s:sensors){
        s.start_ranging();
    }

    while(true) {
        for (auto& s:sensors){
            VL53L5CX_ResultsData data;
            s.get_data(&data);
            enac::Lidar lidar_msg;
            for (int i=0; i<RESULT_LENGTH; i++){
                lidar_msg.add_distances(data.distance_mm[i]);
                lidar_msg.add_angles(i);
                lidar_msg.add_quality(data.reflectance[i]);
            }
            publishers[&s].Send(lidar_msg);
        }
	    usleep(75000);
    }
}
