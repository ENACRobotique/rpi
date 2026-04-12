# Drivers

Ce dossier contient tous les drivers, c'est à dire les programmes qui font l'interface entre le matériel d'un côté, et eCAL de l'autre.
Ces programmes doivent s'exécuter sur la raspberry pi afin d'accèder au matériel (port série, GPIO, I2C, ...).

- `joystick` : Pour commander le robot en manuel avec la manette
- `lidar` : Drivers pour le LD06 en C++, et pour le UST05LN en python
- `smart_servo` : service eCAL qui gère le SAP (smart actuator protocol) : protocole dynamixel 1.0
- `vl53cpp` : VL53L5Cx
- `bridge_base.py` : base roulante
- `tirette` : Utilise les GPIO de la raspi
