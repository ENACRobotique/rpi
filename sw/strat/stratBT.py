#!/usr/bin/env python3
import py_trees
import sys
sys.path.append("../..")
from robot import Robot
from IO.IO_BT import * 
from bt_essentials import *
from math import radians, pi
import functools

def main_bt(robot:Robot):
    
    def position_de_depart(r):
        return START_POS[r.color][r.strat]
    
    def nav_zone_fin(r):
        return END_POS[r.color][r.strat][1]
    
    def destination_finale(r):
        return END_POS[r.color][r.strat][0]
    
    def banderole_recalage(r):
        return Pos(x=r.pos.x, y=105, theta=pi/6)
    
    poserBanderolle = py_trees.composites.Sequence("Poser la banderolle", True)
    poserBanderolle.add_children([
        # MoveTo(Pos(1800,200,pi/6)),
        Bouge(Speed(-60,-200,0),1),
        # Recalage(banderole_recalage),
        #Deplace_toi(-15,-120,100),
        LiftBanderole(False),
        Deplace_toi(-150,-120,300)
    ])

    chercherGradin = py_trees.composites.Sequence("Recherche d'un Gradin", True)

    ramasseGradin = py_trees.composites.Sequence("Rammasser un Gradin", True)
    ramasseGradin.add_children([
        # LiftPlanche(MID),
        AlignPlanches(),
        AlignConserves(),
        AlignPlanches(),
        AlignConserves(),
        # LiftPlanche(DOWN),
        AvancePlanches(),
        # GrabLowConserve(True),
        # LiftPlanche(UP),
        # GrabHighConserve(False),
        # GrabLowConserve(True),
        # LiftConserve(ValeurActionneur.AscenseurAimantUP),
        # MoveRentreur(OUTSIDE),
        # GrabHighConserve(True),
        # GrabLowConserve(False),
        # LockPlanche(True),
        # LiftConserve(ValeurActionneur.AscenseurAimantINTERMEDIAIRE), #2 lignes pas 2 definitives
        # GrabLowConserve(True),
        # LiftConserve(ValeurActionneur.AscenseurAimantDOWN),
        # GrabLowConserve(True),
        # MoveRentreur(INSIDE),
        # Aligne_conserve_seul(Actionneur.AimantBasDroit),
        # AvanceConserve(Actionneur.AimantBasDroit),
        # Aligne_conserve_seul(Actionneur.AimantBasGauche),
        # AvanceConserve(Actionneur.AimantBasGauche),
        # LiftConserve(ValeurActionneur.AscenseurAimantRAISED)
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

    # todo: strat très basique
    basicStrat = py_trees.composites.Sequence("Strat basique", True)
    basicStrat.add_children([
        WaitMatchStart(),
        LiftBanderole(True),
        Recalage(position_de_depart),
        poserBanderolle,
        Navigate(nav_zone_fin),
        EndStrat()
        # DeployMacon(),
        # Recalage(position_de_depart),
        # MoveTo(Pos(800,300,radians(90))),
        # ramasseGradin,
        # chercherGradin,
        # ramasseGradin,
        # chercherDepose,
        # construitGradin,# 2 fois ? 
        ])
    mainBt = py_trees.composites.Selector("Main BT", False)
    mainBt.add_children([
        MatchTimer(85),
        Evitement(),
        basicStrat,
        
    ])
    
    return mainBt


def post_tick_handler(snapshot_visitor, behaviour_tree):
    print(
        py_trees.display.unicode_tree(
            behaviour_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.visited
        )
    )


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

    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    tree.add_post_tick_handler(
    functools.partial(post_tick_handler,
                      snapshot_visitor))
    tree.visitors.append(snapshot_visitor)


    while tree.root.status != py_trees.common.Status.SUCCESS:
        tree.tick()
        time.sleep(0.01)
    print("Fin de l'arbre")
    ecal_core.finalize()
    r.actionneurs.Servo_IO.client.destroy()
    r.locomotion.stop()
