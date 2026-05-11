[Unit]
Description= lidar localisation
After=network.target network-online.target

[Service]
Type=simple

WorkingDirectory=/home/robot/rpi/sw/UCD
ExecStart=/home/robot/rpi/robEnv/bin/python3 /home/robot/rpi/sw/UCD/aruco_UCD.py /dev/ttyUSB0 3

Restart=on-failure
RestartSec=2 
# Configures the time to wait before service is stopped forcefully.
TimeoutStopSec=300
 
[Install]


