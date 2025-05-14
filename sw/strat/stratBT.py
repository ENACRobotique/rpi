#!/usr/bin/env python3
import py_trees
import sys
sys.path.append("../..")
from robot import Robot
from IO.IO_BT import * 
from bt_essentials import*

def main_bt(robot:Robot):
    
    poserBanderolle = py_trees.composites.Sequence("Poser la banderolle", True)
    poserBanderolle.add_children([
        Deplace_toi(80,-120-4,50),
        LiftBanderole(False),
        Deplace_toi(-250,-120-4,100)
    ])

    chercherGradin = py_trees.composites.Sequence("Recherche d'un Gradin", True)

    ramasseGradin = py_trees.composites.Sequence("Rammasser un Gradin", True)
    ramasseGradin.add_children([
        AlignPlanches(),
        AlignConserves(),
        AvancePlanches(),
        GrabLowConserve(True),
        LiftPlanche(UP),
        GrabHighConserve(False),
        GrabLowConserve(True),
        LiftConserve(ValeurActionneur.AscenseurAimantUP),
        MoveRentreur(OUTSIDE),
        GrabHighConserve(True),
        GrabLowConserve(False),
        LockPlanche(True),
        LiftConserve(ValeurActionneur.AscenseurAimantINTERMEDIAIRE), #2 lignes pas 2 definitives
        GrabLowConserve(True),
        LiftConserve(ValeurActionneur.AscenseurAimantDOWN),
        GrabLowConserve(True),
        MoveRentreur(INSIDE),
        Aligne_conserve_seul(Actionneur.AimantBasDroit),
        AvanceConserve(Actionneur.AimantBasDroit),
        Aligne_conserve_seul(Actionneur.AimantBasGauche),
        AvanceConserve(Actionneur.AimantBasGauche),
        LiftConserve(ValeurActionneur.AscenseurAimantRAISED)
    ])

    chercherDepose = py_trees.composites.Sequence("Recherche d'une depose de Gradin", True)

    construitGradin = py_trees.composites.Sequence("Construire un Gradin", True)
    construitGradin.add_children([
        LiftConserve(ValeurActionneur.AscenseurAimantDOWN),
        LiftPlanche(DOWN),
        MoveRentreur(OUTSIDE),
        GrabHighConserve( False),
        MoveRentreur(INSIDE),
        LockPlanche(False),
        GrabLowConserve(False),
    ])

    goZoneDeFin = py_trees.composites.Sequence("Je vais en zone de fin", True)
    
    # todo: strat très basique 
    basicStrat = py_trees.composites.Sequence("Strat basique", True)
    basicStrat.add_children([
        WaitMatchStart(),
        poserBanderolle
        # DeployMacon()
        
        # poserBanderolle,
        # chercherGradin,
        # ramasseGradin,
        # chercherDepose,
        # construitGradin,# 2 fois ? 
        # goZoneDeFin,
        
        ])
    
    mainBt = py_trees.composites.Selector("Main BT", False)
    mainBt.add_children([
        EndMatch(10),
        #Evitement(robot),
        basicStrat
    ])
    
    return mainBt

# === Boucle principale ===
if __name__ == "__main__":
    r = Robot()
    blackboard = py_trees.blackboard.Client(name="Foo Global")
    blackboard.register_key(key="robot", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="matchTime", access=py_trees.common.Access.WRITE)
    blackboard.robot = r
    blackboard.matchTime = 0
    
    tree = py_trees.trees.BehaviourTree(main_bt(r))
    tree.setup(timeout=15)
    while tree.root.status != py_trees.common.Status.SUCCESS:
        tree.tick()
        time.sleep(0.01)
    print("Fin de l'arbre")
    ecal_core.finalize()
    r.actionneurs.Servo_IO.client.destroy()
    r.locomotion.stop()
