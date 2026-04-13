# Logiciels "haut-niveau"

Code haut niveau du robot.
Peut s'éxécuter depuis un ordinateur distant car il ne nécessite pas d'accès au matériel.

- `robot.py` : interface pour commander le robot, et dans laquelle est stockée l'état du robot.
- `world.py` : stocke l'état du monde, pour adapter la stratégie
- `common.py` : contient des classes et fonctions usuelles (Pos, Speed, normalize_angle, ...)
