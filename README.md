# Code haut niveau ENAC Robotique pour Eurobot

Code haut niveau d'ENAC Robotique pour la compétition Eurobot.

Veuillez consulter les [instructions d'installation](conf/Readme.md) pour la configurer sur un Raspberry Pi.

Initialisez les sous-modules avec `git submodule update --init --recursive`.

Ce dépôt repose fortement sur [eCAL](https://ecal.io) pour les communications inter-processus. C'est comme ROS, mais en plus léger et plus facile à utiliser.

## Architecture électronique

Le robot est composé de :
- un Raspberry Pi 5
- une carte "base roulante", commandant les moteurs avec une IMU
- une ou plusieurs carte "IO" pour s'interfacer avec des actionneurs (servos, pompes, ...)
- un écran LCD pour l'interface utilisateur

Certains capteurs sont directement connectés au Raspberry Pi, comme le lidar LD06 et les capteurs de distance VL53L5Cx.

Toutes ces cartes communiquent via UART, voir les [règles udev](conf/_etc_udev_rules.d/80-robot.rules) pour configurer les noms de périphériques sans ambiguïté.

Pour plus d'informations : 
- Carte base roulante [électronique](https://github.com/ENACRobotique/Cartes_Base_roulante) et [code](https://github.com/ENACRobotique/base_roulante_2024)
- [Smart Pumps](https://github.com/ENACRobotique/smart_pump)


## Architecture logicielle

Le code est divisé en plusieurs processus, qui communiquent via un bus de publish-subscribe [eCAL](https://ecal.io) (comme ROS, mais en plus léger et plus facile à utiliser).
Les définitions des messages sont dans le submodule [proto](proto/). 

Les processus sont des services utilisateur systemd. Voir le script d'installation dans le répertoire [services](services/).
Les fichiers de service supposent que le nom d'utilisateur est `robot` et que le chemin vers ce dépôt est `/home/robot/rpi`.

