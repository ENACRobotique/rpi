# RaspberryPi Config

Instructions for setting up a RaspberryPi for the robot.

The overlays and the network can probably be configured before the first boot.


## Overlays (UART, shutdown, bluetooth)

Disable the login shell on UART:
Open `/boot/firmware/cmdline.txt` and remove `console=serial0,115200`

Open /boot/firmware/config.txt and make sure that the UART is enabled:
`enable_uart=1`

At the end of the file, add:

```
# Bluetooth: use miniuart to free the good UART for the robot
dtoverlay=miniuart-bt
core_freq=250

# Shutdown button
dtoverlay=gpio-shutdown,gpio_pin=24

# activate UART 3,4,5
dtoverlay=uart3
dtoverlay=uart4
dtoverlay=uart5
```

Then reboot.

For more details: https://www.raspberrypi.com/documentation/computers/configuration.html#uarts-and-device-tree

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


