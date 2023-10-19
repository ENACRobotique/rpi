
Rpi conf and ros workspace for Eurobot main robot

To add lidar and st usb link :
	sudo cp 80-robot.rules /etc/udev/rules.d/


If rf2o pkg doesn't build and you get a "BOOST_DIR" error,try adding libboost (it did work for me): sudo apt-get install libboost-all-dev
