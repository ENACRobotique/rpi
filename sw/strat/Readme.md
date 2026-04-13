# Stratégie

C'est le programme du match.
C'est aussi un gros bazar, car ce n'est pas facile de trouver une bonne manière d'exprimer les choses.

## Planificateur
Le planificateur (Planner) évalue toutes les actions disponibles et sélectionne celle offrant la récompense maximale, basée sur l'état actuel du robot et du monde.

Les actions représentent des comportements possibles pour le robot.
Chaque action est une classe dérivée de la classe de base Action, conçue pour être exécutée via un behavior tree utilisant la bibliothèque py_trees. 

Méthodes associées aux actions:
- `create_bt(robot: Robot, world: World) -> Behaviour` : Méthode statique qui génère et retourne un behavior tree à exécuter pour cette action.
- `reward(robot: Robot, world: World) -> float` : Méthode statique qui calcule et retourne une valeur de récompense (score) pour cette action, basée sur l'état du robot et du monde.
- `start_cb(robot: Robot, world: World) -> None` : Callback optionnelle appelée au début de l'exécution de l'action.
- `end_cb(robot: Robot, world: World, status: Status) -> None` : Callback optionnelle appelée à la fin de l'exécution, avec le statut de l'action (succès, échec, etc.).

