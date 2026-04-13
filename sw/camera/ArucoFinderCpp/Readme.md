# Aruco Finder cpp 

Détecte les tags Aruco.

## Utilisation

La source vidéo peut provenir d'une caméra, une vidéo,  ou un topic eCAL.

L'argument positionnel `name` est obligatoire : il sert à ouvrir la calibration de la caméra.
Voir le dossier `data/camera_calibrations`.

Le flag optionnel `-d,--display` est valable pour toutes les sources, et permet d'envoyer sur un topic eCAL les images augmentés des tags aruco avec leur ID sur le topic `images_<cam_name>`.

### Caméra

`./arucoFinder <cam_name> -c /dev/video0 [-W width] [-H height] [-f fps] [--fourcc FOURCC]`

l'argument `--fourcc` permet de régler de format vidéo de la caméra.

Les formats et résolutions disponible peuvent être listées avec : `v4l2-ctl -d /dev/video0 --list-formats-ext`.


### Vidéo

`./arucoFinder <cam_name> -v video_file`

### Topic eCAL

`./arucoFinder <cam_name> -t topic_name`


## Build

```
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

