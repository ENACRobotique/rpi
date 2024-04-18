
Rpi conf and ros workspace for Eurobot main robot

## How to use

To clone with submodules do ```git clone --recurse-submodules -j8 [URL] ```
Or if you already cloned the repo you can still add the submodules after by doing : ```git clone --recurse-submodules [URL]```


To add lidar connection see on wich /dev/ttyUSB the lidar is when pluging it injto the pc then give -x permission to this port. Example xith /dev/ttyUSB0 :  ```chmod 777 /dev/ttyUSB0```

Then in 'robot_rpi_2024/robot_ros2_ws/src/ldlidar_stl_ros2/launch/ld06.launch.py' modify the port_name to the previous one ( default is USB0 )

To add lidar and stlink usb connections permanently ( mostly on pi4 ):
```sudo cp 80-robot.rules /etc/udev/rules.d/```
The lidar port name with this config is /dev/lidar.

Move to robot_ros2_ws and do colcon build to build ros packages.

If rf2o pkg doesn't build and you get a "BOOST_DIR" error,try adding libboost (it did work for me): sudo apt-get install libboost-all-dev.
rf2o pkg can take a long time to build (on the pi4 at least) so don't worry if it's stuck for more than 5 mins while building.

if you get the SetuptoolsDeprecationWarning error , verify your version of setuptool ```pip3 list | grep setuptools```
Then change setup tools version to 58.2.0 if it's not: ```pip3 install setuptools==58.2.0```.

Don't forget to source the install/setup.bash file

To use parameters with a launch file :
- `ros2 launch <pkg_name> pkg.launch.py --show-args` to see the possible args
- `ros2 launch <pkg_name> pkg.launch.py param_1:=value param_2:=value ` to pass args 

You can try by connecting 2 lidars and use rviz2