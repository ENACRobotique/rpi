#include "vl53.h"
#include <stdio.h>
#include <unistd.h>
#include <iostream>
extern "C"{
    #include <vl53l5cx_api.h>
}

#define MAX_RETRY 15

#define	CONSUMER "VL53_driver"

int VL53L5CX::init() {
    // turn on sensor
    if(!line) {
        std::cout << "line is NULL!!!";
        return -1;
    }
    gpiod_line_set_value(line, 0);

    usleep(500000);
    int status = vl53l5cx_comms_init(&dev.platform, VL53L5CX_DEFAULT_I2C_ADDRESS);
	if(status)
	{
		printf("VL53L5CX %d comms init failed.\n", addr);
		fflush(stdout);
		return -1;
	}
    status = vl53l5cx_set_i2c_address(&dev, addr);
	if(status) {
		printf("VL53L5CX %d set addr failed, trying with new addr\n", addr);
		fflush(stdout);
		vl53l5cx_comms_init(&dev.platform, addr);
		status = vl53l5cx_set_i2c_address(&dev, addr);
		if(status) {
			printf("VL53L5CX %d set addr failed with new addr!\n", addr);
			fflush(stdout);
		}
	}

    uint8_t isAlive;
    status = vl53l5cx_is_alive(&dev, &isAlive);
	if(!isAlive || status)
	{
		printf("VL53L5CX %d not detected at requested address\n", addr);
		fflush(stdout);
		return status;
	}

	/* (Mandatory) Init VL53L5CX sensor */
	status = vl53l5cx_init(&dev);
	if(status)
	{
		printf("VL53L5CX %d ULD Loading failed\n", addr);
		fflush(stdout);
		return status;
	}

	printf("VL53L5CX %d Ready ! (Version : %s)\n", addr,
			VL53L5CX_API_REVISION);
	fflush(stdout);


	/*********************************/
	/*        Set some params        */
	/*********************************/


    status = vl53l5cx_set_resolution(&dev, VL53L5CX_RESOLUTION_8X8);
	if(status)
	{
		printf("vl53l5cx_set_resolution failed, status %u\n", status);
		fflush(stdout);
		return status;
	}

	/* Set ranging frequency to 2Hz.
	 * Using 4x4, min frequency is 1Hz and max is 60Hz
	 * Using 8x8, min frequency is 1Hz and max is 15Hz
	 */
	status = vl53l5cx_set_ranging_frequency_hz(&dev, 2);
	if(status)
	{
		printf("vl53l5cx_set_ranging_frequency_hz failed, status %u\n", status);
		fflush(stdout);
		return status;
	}

	/* Set target order to closest */
	status = vl53l5cx_set_target_order(&dev, VL53L5CX_TARGET_ORDER_CLOSEST);
	if(status)
	{
		printf("vl53l5cx_set_target_order failed, status %u\n", status);
		fflush(stdout);
		return status;
	}

    state = SENSOR_OK;

    return 0;
}

void VL53L5CX::turn_off(struct gpiod_chip *chip){
    line = gpiod_chip_get_line(chip, gpio);
    if(gpiod_line_request_output(line, CONSUMER, 0)) {
       std::cout << "Error requesting line " << gpio;
       return;
    }
    gpiod_line_set_value(line, 1);
}

void VL53L5CX::start_ranging(){
    if(state == SENSOR_OK) {
        vl53l5cx_start_ranging(&dev);
    }
}

int VL53L5CX::get_data (VL53L5CX_ResultsData* data){
    if(state == SENSOR_OK) {
        // retry up to MAX_RETRY times
        for(int rt=0; rt<MAX_RETRY; rt++) {
            uint8_t isReady;
            int status = vl53l5cx_check_data_ready(&dev, &isReady);
            if(!status && isReady){
                if (vl53l5cx_get_ranging_data(&dev, data) == VL53L5CX_STATUS_OK){
                    return 0;
                }
            }
            WaitMs(&dev.platform, 10);
        }
    }
    return -1;
}