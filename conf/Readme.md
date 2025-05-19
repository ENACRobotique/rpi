# RaspberryPi Config

Instructions for setting up a RaspberryPi for the robot.

The overlays and the network can probably be configured before the first boot.

To begin, use the rpi imager to flash an SD card with Ubuntu 24.04.

## Get this repository

Clone this repository in the home directory:

git clone --recurse-submodules git@github.com:ENACRobotique/rpi.git

## Overlays (UART, shutdown, bluetooth)

Disable the login shell on UART:
Open `/boot/firmware/cmdline.txt` and remove `console=serial0,115200`

Open /boot/firmware/config.txt and make sure that the UART is enabled:
`enable_uart=1`

At the end of the file, add:

```
# will be enabled for all rpi hardware
[all]

# Bluetooth: use miniuart to free the good UART for the robot
dtoverlay=miniuart-bt
core_freq=250

# Shutdown button
dtoverlay=gpio-shutdown,gpio_pin=24

# will be enabled only  for rpi 4
[pi4]

# activate UART 3,4,5
dtoverlay=uart3
dtoverlay=uart4
dtoverlay=uart5

# will be enabled only for rpi 5
[pi5]

# activate UART 0,1,2,3,4
# Enable uart 0 on GPIOs 14-15
dtoverlay=uart0-pi5
# Enable uart 2 on GPIOs 4-5
dtoverlay=uart2-pi5
# Enable uart 3 on GPIOs 8-9 with CTSRTS for halduplex direction
dtoverlay=uart3-pi5,ctsrts
# Enable uart 4 on GPIOs 12-13
dtoverlay=uart4-pi5
# SPI0 conflicts with UART3
dtparam=spi=off
# Enable RTC backup battery charging
dtparam=rtc_bbat_vchg=3000000

# a tester sans cette ligne, on ne sait pas si c'est utile
gpu_mem=128

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

## UDEV rules

Copy `80-robot.rules` to `/etc/udev/rules.d/`, and edit it to change the configuration according to your setup (pi4 vs pi5, ...)


## Packages

Run `install_packages.sh` to install all necessary packages.

## Build drivers

Run `../build.sh`

## Python venv

Run `setup_venv.sh` to setup the python virtual environnement and install the dependencies.

## Manette Bluetooth

```
sudo bluetoothctl
> scan on
# Appuyer 3s sur le bouton share (en haut à droite de la croix gauche) et le bouton home (entre les joysticks). La manette doit flasher en blanc.
> trust 98:B6:E9:84:66:07
> pair 98:B6:E9:84:66:07
> connect 98:B6:E9:84:66:07
```


