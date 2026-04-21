#!/usr/bin/env python3
from PyQt6 import QtWidgets
import sys
import argparse

from lidar.radar_view import RadarView
    
if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--lidar", default="lidar_data", help="LidarData topic name")
    parser.add_argument("-l", "--no-loca", action="store_false", default=True, help="Do not draw localisation helpers")
    parser.add_argument("-p", "--pi", action="store_true", default=False, help="run with the linuxfb platform")
    args = parser.parse_args()

    if args.pi:
        sys.argv.extend(["-platform", "linuxfb"])

    qapp = QtWidgets.QApplication(sys.argv)
    main_window = QtWidgets.QMainWindow()
    central_widget = QtWidgets.QWidget()
    main_window.setCentralWidget(central_widget)
    layout = QtWidgets.QVBoxLayout(central_widget)

    radarView = RadarView(args.lidar, args.no_loca)
    layout.addWidget(radarView)

    if args.pi:
        main_window.showFullScreen()
    else:
        main_window.show()
    #app.activateWindow()
    #app.raise_()
    qapp.exec()
