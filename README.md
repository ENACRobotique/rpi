
Rpi conf and ros workspace for Eurobot main robot

move to robot_ros2_ws and do colcon build to build ros packages.

If rf2o pkg doesn't build and you get a "BOOST_DIR" error,try adding libboost (it did work for me): sudo apt-get install libboost-all-dev
rf2o pkg can take a long time to build ( on the pi4 at least) so don't worry if it's stuck for more thant 2 mins while building

To add lidar and stlink usb connections:
```sudo cp 80-robot.rules /etc/udev/rules.d/```

Then in 'robot_rpi_2024/robot_ros2_ws/src/ldlidar_stl_ros2/launch/ld06.launch.py' modify the port_name to /dev/lidar, you may want to build again after that

Don't forget to source the install/setup.bash file
