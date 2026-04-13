# Lidar

Traitement des données du lidar

- `amalgameur.py` : Groupe les points LIDAR pour en sortir des "amalgames".
- `loca_lidar.py` : Localisation lidar Voir plus de détails sur [la doc](https://enacrobotique.github.io/doc/to_robot/localisation_lidar/loca_lidar.html).
- `radar_view.py` : Widget Qt6 affichant les données LIDAR.
- `ekf.py` : Implémentation d'un Extended-Kalman-Filter pour fusionner la position lidar et les données du gyro et des encodeurs moteurs.
