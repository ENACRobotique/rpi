# Smart Actuators

Interface pour utiliser les divers actionneurs compatibles avec le "Smart Actuator Protocol", a.k.a le protocole Dynamixel 1.0.

La classe `SAPMaster` du module `sap_master.py` centralise les interfaces des divers types d'actionneurs.

Chaque type d'actionneur à son module python associé, contenant sa memory table, et ses méthodes propres.

Cette interface utilise le service eCAL fourni par le driver `drivers/smart_servo`, qui doit donc être préalablement lancé.

Exemple :

```python
from sap_master import SAPMaster
s = SAPMaster()
s.ping(15)
pos = s.sts3032.read_pos(15)
s.pump.pump(42, 1)
```
