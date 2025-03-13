# VL53L5CX driver

## Build

```
mkdir build && cd build
cmake ..
make
```

Comment/uncomment the relevant sensors in `main.cpp`:

```C++
std::array sensors = {
        VL53L5CX(17, 0x30),
        VL53L5CX(18, 0x32),
        // VL53L5CX(26, 0x34),
        // VL53L5CX(16, 0x36),
    };
```

## GPIO

Ce driver utilise ligpiod.
- Detecter les chips GPIO: `sudo gpiodetect`.
- Récupérer les infos sur le chip: `sudo gpioinfo gpiochip4`.
