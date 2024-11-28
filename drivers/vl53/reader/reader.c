#include <unistd.h>
#include <signal.h>
#include <dlfcn.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>

#include "vl53l5cx_api.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53l5cx_api.h"
#include <stdbool.h>
#include "pigpiod_if2.h"

enum SensorState {
    SENSOR_DOWN,
    SENSOR_OK,
};

typedef struct
{
    uint16_t port;
    uint8_t gpio;
    uint8_t addr;
    enum SensorState state;
    VL53L5CX_Configuration dev;
} sensor_t;

#define MAX_RETRY 15

#define NB_SENSORS 4

sensor_t SENSORS[NB_SENSORS] = {
    {.port=2001, .gpio=26, .addr=0x30, .state=SENSOR_DOWN},
    {.port=2002, .gpio=16, .addr=0x32, .state=SENSOR_DOWN},
    {.port=2003, .gpio=17, .addr=0x34, .state=SENSOR_DOWN},
    {.port=2004, .gpio=18, .addr=0x36, .state=SENSOR_DOWN},
};

int socket_desc;
struct sockaddr_in server_addr;

int pi; // GPIO

int init_sensor(sensor_t* sensor) {
    // turn on sensor
    gpio_write(pi, sensor->gpio, PI_LOW);
    time_sleep(0.5);
    int status = vl53l5cx_comms_init(&sensor->dev.platform, VL53L5CX_DEFAULT_I2C_ADDRESS);
	if(status)
	{
		printf("VL53L5CX %d comms init failed.\n", sensor->port);
		fflush(stdout);
		return -1;
	}
    status = vl53l5cx_set_i2c_address(&sensor->dev, sensor->addr);
	if(status) {
		printf("VL53L5CX %d set addr failed, trying with new addr\n", sensor->port);
		fflush(stdout);
		vl53l5cx_comms_init(&sensor->dev.platform, sensor->addr);
		status = vl53l5cx_set_i2c_address(&sensor->dev, sensor->addr);
		if(status) {
			printf("VL53L5CX %d set addr failed with new addr!\n", sensor->port);
			fflush(stdout);
		}
	}

    uint8_t isAlive;
    status = vl53l5cx_is_alive(&sensor->dev, &isAlive);
	if(!isAlive || status)
	{
		printf("VL53L5CX %d not detected at requested address\n", sensor->port);
		fflush(stdout);
		return status;
	}

	/* (Mandatory) Init VL53L5CX sensor */
	status = vl53l5cx_init(&sensor->dev);
	if(status)
	{
		printf("VL53L5CX %d ULD Loading failed\n", sensor->port);
		fflush(stdout);
		return status;
	}

	printf("VL53L5CX %d Ready ! (Version : %s)\n", sensor->port,
			VL53L5CX_API_REVISION);
	fflush(stdout);


	/*********************************/
	/*        Set some params        */
	/*********************************/


    status = vl53l5cx_set_resolution(&sensor->dev, VL53L5CX_RESOLUTION_8X8);
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
	status = vl53l5cx_set_ranging_frequency_hz(&sensor->dev, 2);
	if(status)
	{
		printf("vl53l5cx_set_ranging_frequency_hz failed, status %u\n", status);
		fflush(stdout);
		return status;
	}

	/* Set target order to closest */
	status = vl53l5cx_set_target_order(&sensor->dev, VL53L5CX_TARGET_ORDER_CLOSEST);
	if(status)
	{
		printf("vl53l5cx_set_target_order failed, status %u\n", status);
		fflush(stdout);
		return status;
	}

    sensor->state = SENSOR_OK;

    return 0;
}

int main(int argc, char ** argv)
{
    // printf("Launching VL53 driver on port %u\n", port);
    // fflush(stdout);	

    // Create sockets
    socket_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(socket_desc < 0){
        printf("Error while creating socket\n");
        fflush(stdout);
        return -1;
    }
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    //server_addr.sin_port = htons(port);

    
    // start gpio
    pi = pigpio_start(0, 0);
    //turn off all sensors
    for(int i=0; i<NB_SENSORS; i++) {
        set_mode(pi, SENSORS[i].gpio, PI_OUTPUT);
        gpio_write(pi, SENSORS[i].gpio, PI_HIGH);
    }

    for(int i=0; i<NB_SENSORS; i++) {
        init_sensor(&SENSORS[i]);
    }

    for(int i=0; i<NB_SENSORS; i++) {
        if(SENSORS[i].state == SENSOR_OK) {
            vl53l5cx_start_ranging(&SENSORS[i].dev);
        }
    }


    while(true) {
        for(int i=0; i<NB_SENSORS; i++) {
            if(SENSORS[i].state == SENSOR_OK) {
                // retry up to MAX_RETRY times
                for(int rt=0; rt<MAX_RETRY; rt++) {
                    uint8_t isReady;
                    int status = vl53l5cx_check_data_ready(&SENSORS[i].dev, &isReady);
                    if(!status && isReady)
                    {
                        VL53L5CX_ResultsData 	Results;
                        vl53l5cx_get_ranging_data(&SENSORS[i].dev, &Results);
                        server_addr.sin_port = htons(SENSORS[i].port);
                        sendto(socket_desc, (uint8_t*)Results.distance_mm, sizeof(Results.distance_mm), 0,(struct sockaddr*)&server_addr, sizeof(server_addr));
                        break;
                    }
                    WaitMs(&SENSORS[i].dev.platform, 10);
                }
            }
        }
	    WaitMs(&SENSORS[0].dev.platform, 75);
    }

}
