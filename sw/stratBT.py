import py_trees
import sys
sys.path.append("../..")
from robot import Robot
from IO.IO_BT import * 
from bt_essentials import*

def main_bt(robot:Robot):
    
    poserBanderolle = py_trees.composites.Sequence("Poser la banderolle", True)

    chercherGradin = py_trees.composites.Sequence("Recherche d'un Gradin", True)


    ramasseGradin = py_trees.composites.Sequence("Rammasser un Gradin", True)
    ramasseGradin.add_children([
        LiftPlanche(robot.actionneurs, UP),
        WaitSeconds(0.5),
        GrabHighConserve(False),
        GrabLowConserve(True),
        WaitSeconds(0.3),
        LiftConserve(robot.actionneurs, UP),
        WaitSeconds(2),
        MoveRentreur(robot.actionneurs, OUTSIDE),
        WaitSeconds(0.1),
        GrabHighConserve(robot.actionneurs, True),
        WaitSeconds(1.5),
        GrabLowConserve(robot.actionneurs, False),
        WaitSeconds(0.25),
        LockPlanche(robot.actionneurs, True),
        LiftConserve(robot.actionneurs, DOWN),
        WaitSeconds(0.5),

        GrabLowConserve(robot.actionneurs, True),
        WaitSeconds(0.7),

        MoveRentreur(robot.actionneurs, INSIDE),
        WaitSeconds(0.3),
        GrabLowConserve(robot.actionneurs, False)
    ])

    chercherDepose = py_trees.composites.Sequence("Recherche d'une depose de Gradin", True)

    construitGradin = py_trees.composites.Sequence("Construire un Gradin", True)
    construitGradin.add_children([
            LiftPlanche(robot.actionneurs,DOWN),
            WaitSeconds(2.5),
            MoveRentreur(robot.actionneurs, OUTSIDE),
            WaitSeconds(1.8),
            GrabHighConserve(robot.actionneurs, False),
            WaitSeconds(0.3),
            MoveRentreur(robot.actionneurs, INSIDE),
            WaitSeconds(0.5),
            LockPlanche(robot.actionneurs, False),
            WaitSeconds(0.5),
            GrabLowConserve(robot.actionneurs, False) 
    ])

    goZoneDeFin = py_trees.composites.Sequence("Je vais en zone de fin", True)
    
    # todo: strat très basique 
    basicStrat = py_trees.composites.Sequence("Strat basique", True)
    basicStrat.add_children([
        poserBanderolle,
        chercherGradin,
        ramasseGradin,
        chercherDepose,
        construitGradin,# 2 fois ? 
        goZoneDeFin,
        EndMatch()])
    
    mainBt = py_trees.composites.Selector("Main BT", False)
    mainBt.add_children([
        Evitement(robot)
    ])
    
    return mainBt

# === Boucle principale ===
if __name__ == "__main__":
    r = Robot()
    tree = py_trees.trees.BehaviourTree(main_bt(r))
    tree.setup(timeout=15)

    while True:
        tree.tick()
        time.sleep(0.1)
