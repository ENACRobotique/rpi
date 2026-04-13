# Calibrations caméras

Ce dossier contient les calibrations des caméras, ainsi que les positions des caméras par rapport au repère du robot.


Les noms de fichiers de calibration suivent ce format : `<cam_name>_<width>x<height>.yml`
La position / orientation des caméra : `<cam_name>_tvec.yml` et `<cam_name>_rvec.yml`.

La calibration se fait avec le script `tools/camera/calibrateCamera.py`.
La calibration de la pose se fait avec le script `tools/camera/camera_pos.py`.

