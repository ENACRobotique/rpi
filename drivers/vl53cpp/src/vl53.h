#pragma once
#include <inttypes.h>
extern "C"{
    #include <vl53l5cx_api.h>
}
#include <gpiod.h>

enum SensorState {
    SENSOR_DOWN,
    SENSOR_OK,
};

class VL53L5CX { 
public :
    VL53L5CX(uint8_t gpio, uint8_t addr):gpio(gpio), line(NULL), addr(addr),state(SENSOR_DOWN){}
    int init();
    void turn_off(struct gpiod_chip *chip);
    void start_ranging();
    int get_data (VL53L5CX_ResultsData* data);

private :
    uint8_t gpio;
    uint8_t addr;
    struct gpiod_line* line;
    enum SensorState state;
    VL53L5CX_Configuration dev;
    
};
