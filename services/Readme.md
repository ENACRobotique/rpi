# Services

Services systemd pour lancer automatiquement au démarrage les programmes nécéssaires au robot.

## Configuration

Les services s'exécutent sous l'utilisateur `robot` (et non en root).

Il faut donc les gérer avec `systemctl --user`.

Exemple :
`systemctl --user status robot_bridge.service`

Le script `install_service.sh` permet de réinstaller les services après modification.
Attention, il ne redémarre pas les services après-coup.

## Liste des services

- `robot_bridge.service` : Bridge base roulante
- `robot_lidar_driver.service` : Driver LIDAR
- `robot_joystick.service` : Driver joystick
- `robot_IO.service` : Driver SmartServos
- `robot_vl53.service` : Driver VL53L5Cx
- `robot_aruco.service` : Détecteur de tags Aruco
- `robot_aruco2.service` : Détecteur de tags Aruco avec une autre caméra
- `robot_lidar_amalgameur.service` : Détecte les objets avec les points lidar. À intégrer dans la loca lidar.
- `robot_lidar_loca.service` : Localisation LIDAR
- `robot_ui.service` : Interface utilisateur pour l'écran LCD
- `robot_strat.service` : Strat du robot. Le programme qui gère le match, "l'intelligence" du robot.
- `robot_start.service` : Méta service qui démarre tout les autres
