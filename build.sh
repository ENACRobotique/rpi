#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

ld06_driver_path=$SCRIPT_DIR/drivers/lidar/ld06_cpp
vl53_driver_path=$SCRIPT_DIR/drivers/vl53cpp
servos_driver_path=$SCRIPT_DIR/drivers/smart_servo
ui_path=$SCRIPT_DIR/sw/ui

rm -r $ld06_driver_path/build
rm -r $vl53_driver_path/build
rm -r $servos_driver_path/build

# exit on error
set -e

mkdir -p $ld06_driver_path/build
cmake -S $ld06_driver_path -B $ld06_driver_path/build
cmake --build $ld06_driver_path/build

mkdir -p $vl53_driver_path/build
cmake -S $vl53_driver_path -B $vl53_driver_path/build
cmake --build $vl53_driver_path/build

mkdir -p $servos_driver_path/build
cmake -S $servos_driver_path -B $servos_driver_path/build
cmake --build $servos_driver_path/build

mkdir -p $ui_path/build
cmake -S $ui_path -B $ui_path/build
cmake --build $ui_path/build
