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


int example2(VL53L5CX_Configuration *p_dev);
int exit_main_loop = 0;

void sighandler(int signal)
{
	printf("SIGNAL Handler called, signal = %d\n", signal);
	exit_main_loop  = 1;
}

//sockets
int socket_desc;
struct sockaddr_in server_addr;

int main(int argc, char ** argv)
{
	int status;
	VL53L5CX_Configuration 	Dev;


    // Create sockets
    socket_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(socket_desc < 0){
        printf("Error while creating socket\n");
        return -1;
    }
	server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(2000);
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* Initialize channel com */
	status = vl53l5cx_comms_init(&Dev.platform);
	if(status)
	{
		printf("VL53L5CX comms init failed\n");
		return -1;
	}

	status = example2(&Dev);


	vl53l5cx_comms_close(&Dev.platform);

	return 0;
}






int example2(VL53L5CX_Configuration *p_dev)
{

	/*********************************/
	/*   VL53L5CX ranging variables  */
	/*********************************/

	uint8_t 				status, isAlive, isReady;
	//uint32_t 				integration_time_ms;
	VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */


	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53L5CX sensor connected */
	status = vl53l5cx_is_alive(p_dev, &isAlive);
	if(!isAlive || status)
	{
		printf("VL53L5CX not detected at requested address\n");
		return status;
	}

	/* (Mandatory) Init VL53L5CX sensor */
	status = vl53l5cx_init(p_dev);
	if(status)
	{
		printf("VL53L5CX ULD Loading failed\n");
		return status;
	}

	printf("VL53L5CX ULD ready ! (Version : %s)\n",
			VL53L5CX_API_REVISION);
			

	/*********************************/
	/*        Set some params        */
	/*********************************/

	/* Set resolution in 8x8. WARNING : As others settings depend to this
	 * one, it must be the first to use.
	 */
	status = vl53l5cx_set_resolution(p_dev, VL53L5CX_RESOLUTION_8X8);
	if(status)
	{
		printf("vl53l5cx_set_resolution failed, status %u\n", status);
		return status;
	}

	/* Set ranging frequency to 2Hz.
	 * Using 4x4, min frequency is 1Hz and max is 60Hz
	 * Using 8x8, min frequency is 1Hz and max is 15Hz
	 */
	status = vl53l5cx_set_ranging_frequency_hz(p_dev, 2);
	if(status)
	{
		printf("vl53l5cx_set_ranging_frequency_hz failed, status %u\n", status);
		return status;
	}

	/* Set target order to closest */
	status = vl53l5cx_set_target_order(p_dev, VL53L5CX_TARGET_ORDER_CLOSEST);
	if(status)
	{
		printf("vl53l5cx_set_target_order failed, status %u\n", status);
		return status;
	}

	// /* Get current integration time */
	// status = vl53l5cx_get_integration_time_ms(p_dev, &integration_time_ms);
	// if(status)
	// {
	// 	printf("vl53l5cx_get_integration_time_ms failed, status %u\n", status);
	// 	return status;
	// }
	// printf("Current integration time is : %d ms\n", integration_time_ms);


	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53l5cx_start_ranging(p_dev);

	while(1)
	{
		/* Use polling function to know when a new measurement is ready.
		 * Another way can be to wait for HW interrupt raised on PIN A3
		 * (GPIO 1) when a new measurement is ready */
 
		status = vl53l5cx_check_data_ready(p_dev, &isReady);

		if(isReady)
		{
			vl53l5cx_get_ranging_data(p_dev, &Results);
			sendto(socket_desc, (uint8_t*)Results.distance_mm, sizeof(Results.distance_mm), 0,(struct sockaddr*)&server_addr, sizeof(server_addr));
		}

		/* Wait a few ms to avoid too high polling (function in platform
		 * file, not in API) */
		WaitMs(&p_dev->platform, 5);
	}

	status = vl53l5cx_stop_ranging(p_dev);
	printf("End of ULD demo\n");
	return status;
}
