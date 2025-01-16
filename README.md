# High level code for eurobot robot

ENAC Robotic's high level code for the eurobot competition.

Please see the [installation instructions](conf/Readme.md) to set it up on a Raspberry Pi.

Init the submodules with `git submodule update --init --recursive`.

This repo heavily relies on [eCAL](https://ecal.io) for inter-process communications. Think about it like ROS, but lighter and easier to use.

## Electronic architecture

The robot is composed of:
- a Raspberry Pi 4 (or 5)
- "base roulante" board, driving the motors and featuring an IMU
- The IO board to handle actuators and some sensors
- The LCD board, a simple UI.

Some sensors are directly connected to the Raspberry Pi, like the LD06 lidar, and the 8x8 range sensors VL53L5Cx.

All theses boards communicate via UART, see the [udev rules](conf/80-robot.rules) to setup unambiguous device names.


## Software architecture

The code is divided in multiple processes, that communicate over an [eCAL](https://ecal.io) publish-subscribe bus.
The messages definitions are in the [proto](proto/) submodule. 

The processes are systemd user services. See the install script in the [services](services/) directory.
The services unit files assume that the username is `robot` and the path to this repository is `/home/robot/rpi`.


Manage the service with `systemctl --user status robot_*.services`.



--------


requirements : 
- heapq
- typing
- ...

# How to use services

You can manage services by using systemd 
Do `` sudo systemctl <option> <service> `` to interact.

Options : 
- start
- stop
- status
- restart
- enable
- disable

All robot necessary services being with the prefix "robot_". You can use TAB to list them all while writing the command.
You can edit the install_service.sh and run if you make new services.
To edit new services read the associated documentation. You may also just copy paste a working service and hope it works :).


