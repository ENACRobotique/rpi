import py_trees
import sys
sys.path.append("../..")
from robot import Robot
from IO.IO_BT import * 
from bt_essentials import*

def main_bt(jerome: IO_Manager):
    
    poserBanderolle = py_trees.composites.Sequence("Poser la banderolle", True)

    chercherGradin = py_trees.composites.Sequence("Recherche d'un Gradin", True)


    ramasseGradin = py_trees.composites.Sequence("Rammasser un Gradin", True)
    ramasseGradin.add_children([
        LiftPlanche(jerome, UP),
        WaitSeconds(0.5),
        GrabHighConserve(jerome, False),
        GrabLowConserve(jerome, True),
        WaitSeconds(0.3),
        LiftConserve(jerome, UP),
        WaitSeconds(2),
        MoveRentreur(jerome, OUTSIDE),
        WaitSeconds(0.1),
        GrabHighConserve(jerome, True),
        WaitSeconds(1.5),
        GrabLowConserve(jerome, False),
        WaitSeconds(0.25),
        LockPlanche(jerome, True),
        LiftConserve(jerome, DOWN),
        WaitSeconds(0.5),

        GrabLowConserve(jerome, True),
        WaitSeconds(0.7),

        MoveRentreur(jerome, INSIDE),
        WaitSeconds(0.3),
        GrabLowConserve(jerome, False)
    ])

    chercherDepose = py_trees.composites.Sequence("Recherche d'une depose de Gradin", True)

    construitGradin = py_trees.composites.Sequence("Construire un Gradin", True)
    construitGradin.add_children([
            LiftPlanche(jerome,DOWN),
            WaitSeconds(2.5),
            MoveRentreur(jerome, OUTSIDE),
            WaitSeconds(1.8),
            GrabHighConserve(jerome, False),
            WaitSeconds(0.3),
            MoveRentreur(jerome, INSIDE),
            WaitSeconds(0.5),
            LockPlanche(jerome, False),
            WaitSeconds(0.5),
            GrabLowConserve(jerome, False) 
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
        Evitement(),
        basicStrat
    ])
    
    return basicStrat



def joystick_bt(jerome: IO_Manager):
    """Joystick will use this"""
    
    ramasseGradin = py_trees.composites.Sequence("Rammasser un Gradin", True)
    ramasseGradin.add_children([
        LiftPlanche(jerome, UP),
        WaitSeconds(0.5),
        GrabHighConserve(jerome, False),
        GrabLowConserve(jerome, True),
        WaitSeconds(0.3),
        LiftConserve(jerome, UP),
        WaitSeconds(2),
        MoveRentreur(jerome, OUTSIDE),
        WaitSeconds(0.1),
        GrabHighConserve(jerome, True),
        WaitSeconds(1.5),
        GrabLowConserve(jerome, False),
        WaitSeconds(0.25),
        LockPlanche(jerome, True),
        LiftConserve(jerome, DOWN),
        WaitSeconds(0.5),

        GrabLowConserve(jerome, True),
        WaitSeconds(0.7),

        MoveRentreur(jerome, INSIDE),
        WaitSeconds(0.3),
        GrabLowConserve(jerome, False)
    ])

    construitGradin = py_trees.composites.Sequence("Construire un Gradin", True)
    construitGradin.add_children([
            LiftPlanche(jerome,DOWN),
            WaitSeconds(2.5),
            MoveRentreur(jerome, OUTSIDE),
            WaitSeconds(1.8),
            GrabHighConserve(jerome, False),
            WaitSeconds(0.3),
            MoveRentreur(jerome, INSIDE),
            WaitSeconds(0.5),
            LockPlanche(jerome, False),
            WaitSeconds(0.5),
            GrabLowConserve(jerome, False) 
    ])

    root = py_trees.composites.Sequence("Root", True)
    root.add_children([
        waitCalibration(jerome),
        DeployMacon(jerome),
        WaitSeconds(2),
        ramasseGradin])
    return root

