# Smart servo driver

Driver pour le protocol [Dynamixel 1.0](https://emanual.robotis.com/docs/en/dxl/protocol1/).

Expose un service eCAL avec les méthodes `ping` `read_reg` et `write_reg`.

S'utilise avec l'interface python `sw/actuators/sap_master.py`.

Compatible avec: 
- STS3032
- AX12A
- SCS0009
- smart pumps ENAC Robotique
- et bien plus...

# Compilation

```
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

