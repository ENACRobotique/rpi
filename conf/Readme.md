# RaspberryPi Config

Instructions for setting up a RaspberryPi for the robot.

The overlays and the network can probably be configured before the first boot.

To begin, use the rpi imager to flash an SD card with Ubuntu 24.04.


## Overlays (UART, shutdown, bluetooth)

Disable the login shell on UART:
Open `/boot/firmware/cmdline.txt` and remove `console=serial0,115200`

Open /boot/firmware/config.txt and make sure that the UART is enabled:
`enable_uart=1`

At the end of the file, add:

```
[all] # will be enabled for all rpi hardware

# Bluetooth: use miniuart to free the good UART for the robot
dtoverlay=miniuart-bt
core_freq=250

# Shutdown button
dtoverlay=gpio-shutdown,gpio_pin=24


[pi4] # will be enabled only  for rpi 4

# activate UART 3,4,5
dtoverlay=uart3
dtoverlay=uart4
dtoverlay=uart5


[pi5] # will be enabled only for rpi 5

# activate UART 0,1,2,3,4
dtoverlay=uart0-pi5 # Enable uart 0 on GPIOs 14-15
#dtoverlay=uart1-pi5 # Enable mini-uart 1 on GPIOs 0-1 
dtoverlay=uart2-pi5 # Enable uart 2 on GPIOs 4-5
dtoverlay=uart3-pi5 # Enable uart 3 on GPIOs 8-9
dtoverlay=uart4-pi5 # Enable uart 4 on GPIOs 12-13

```

Then reboot.

For more details: 
- https://www.raspberrypi.com/documentation/computers/configuration.html#uarts-and-device-tree 
- https://www.raspberrypi.com/documentation/computers/config_txt.html#conditional-filters
- https://github.com/Felipegalind0/RPI5.pinout   Chapter "dtoverlays for UARTs"
- https://gist.github.com/lbussy/9e81cbcc617952f1250e353bd42e7775 Chapter "Boot Overlay"


## Network

```
sudo apt install network-manager
```

- Copy `50-ENAC-network.yaml` in /etc/netplan/
- Edit /etc/netplan/50-ENAC-network.yaml to set the correct wifi SSID and password.
- run `netplan apply`.


## Packages

Run `install_packages.sh` to install all necessary packages.


## Python path

To add directories to python path, copy `robot_enac.pth` to `/usr/lib/python3/dist-packages/`:

`cp robot_enac.pth /usr/lib/python3/dist-packages/`


## Manette Bluetooth

Need the `bluez` package.

```
sudo apt install bluez
sudo bluetoothctl
> scan on
# Appuyer 3s sur le bouton share (en haut à droite de la croix gauche) et le bouton home (entre les joysticks). La manette doit flasher en blanc.
> trust 98:B6:E9:84:66:07
> pair 98:B6:E9:84:66:07
> connect 98:B6:E9:84:66:07
```


## GPIO for C

```
mkdir -p ~/lib
cd ~/lib
git clone https://github.com/joan2937/pigpio
cd pigpio
make
sudo make install
```

May need `python3-setuptools`

More details: https://abyz.me.uk/rpi/pigpio/download.html


