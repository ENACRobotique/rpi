# High level code for eurobot 2024 robot

This repo contains submodules.
To clone with submodules do ```git clone --recurse-submodules -j8 [URL] ```
Or if you already cloned the repo you can still add the submodules after by doing : ```git clone --recurse-submodules [URL]```

To add lidar and stlink usb connections permanently ( mostly on pi4 ) use the .rule in conf directory : ( this may change due to rpi4 shield using uart !)
```sudo cp 80-robot.rules /etc/udev/rules.d/```
The lidar port name with this config is /dev/lidar.
The base roulante name with this config is /dev/bas_niveau

requirements : 
- [ecal](https://eclipse-ecal.github.io/ecal/index.html)
- protobuff (version ?)
- pyserial 3.5 
- numpy
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
