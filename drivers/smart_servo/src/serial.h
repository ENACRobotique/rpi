#pragma once
#include <stdint.h>
#include <stddef.h>
#include <termios.h>

int init_serial(int fd, speed_t speed);
void initDriver(int gpio);
void enableDriver(int fd, bool enable);
int writeData(int fd, uint8_t* data, size_t len, bool echo);
