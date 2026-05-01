from planner import Action
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.composites import Selector, Sequence
import sys
import numpy as np
sys.path.append("../..")
from robot import Robot, COTE_DROIT, COTE_GAUCHE, Velocity
from common import Speed
from world import World,RAMASSAGE_POS,DEPOT_POS,DEPOT_ANG,RAMASSAGE_ANG
from bt_essentials import MatchTimer, Navigate, WaitMatchStart, WaitUntil
from bt_essentials import EndStrat, END_POS, WaitSeconds, THERMO_POS, MoveTo, Move, START_POS, CAISSETHERMO_POS
from typing import Callable
from dataclasses import dataclass
import time
import subprocess
from IO.IO_BT import *

DISTANCE_MAX  = np.sqrt(13) * 1000 # en mm
SEUIL_AGRESSIVITE = 3 # Compris entre 4 (Ghandi) et 2 (Chabal)

##################################
###      Action Thermometre    ###
##################################

class ThermometreAction(Action):
    name = "Thermometre"

    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        nav_pt = THERMO_POS[robot.color][robot.strat]
        nav_pt2 = CAISSETHERMO_POS[robot.color][robot.strat]
        cote = False if robot.color == Team.JAUNE else True # ie on recup cote Gauche avec le jaune pour avoir bras droit libre (et inversement cote bleu)

        def thermo_point(_):
            return nav_pt
        def caissethermo_point(_):
            return nav_pt2
        
        bougerThermo = py_trees.composites.Sequence("Thermometre", True)
        bougerThermo.add_children([
            WaitSeconds(0.5),
            Navigate(caissethermo_point),
            Aligner(cote),
            Attraper(cote),
            Navigate(thermo_point),
            #MoveTo(robot.dest_to_pos(CAISSETHERMO_POS[robot.color][robot.strat])),
            ThermoAction(THERMO_POS[robot.color][robot.strat])
        ])
        return bougerThermo
    
    @staticmethod
    #POUSSE = 2
    def reward(robot: Robot, world: World) -> float:
        if world.thermo_positioned:
            return 0
        else:
            return 10
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        RAMASSAGE_POS[CAISSETHERMO_POS[robot.color][robot.strat][0]]=False # On dit qu'on a recup la caisse
        if status == py_trees.common.Status.SUCCESS:
            world.thermo_positioned = True
            robot.updateScore(10)


class Recuperer(Action):
    name = "Recuperer"
    nav_point = "NAN"

    @staticmethod
    def recup_point(_,cote):
        angle = RAMASSAGE_ANG[Recuperer.nav_point]  if cote else RAMASSAGE_ANG[Recuperer.nav_point]+ np.pi
        return (Recuperer.nav_point,angle)
    
    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        recup = py_trees.composites.Sequence("Recuperer", True)
        cote = True if robot.cote_droit_vide() else False

        recup.add_children([
            WaitSeconds(0.5),
            Navigate(lambda x : Recuperer.recup_point(x,cote)), 
            Aligner(cote),
            Attraper(cote)
        ])
        return recup
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        if robot.cote_droit_vide() or robot.cote_gauche_vide():
            # fonction qui calcule le point de ramasse qui rapporte le plus
            def mostRewardingGrabPoint():
                max_reward = -1
                max_wpt = "NAN"
                for wpt in RAMASSAGE_POS.keys():
                    if RAMASSAGE_POS[wpt]:
                        # On a des valeurs qui dependent de la distance, le gain minimal est 3 (a la distance max theorique) jusqu'a 6.
                        val = 6  - 3 * (robot.distance_from(wpt)/DISTANCE_MAX) #- SEUIL_AGRESSIVITE * (distance robot adverse/ distanceMax)
                    else :
                        val = 0
                    if max_reward < val:
                        max_reward = val
                        max_wpt = wpt
                return max_wpt,max_reward
        
            Recuperer.nav_point, max_reward = mostRewardingGrabPoint()
            return max_reward
        else:
            return 0
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        if status == py_trees.common.Status.SUCCESS:
            if Recuperer.nav_point == "NoixJEN" or Recuperer.nav_point == "NoixJES":
                RAMASSAGE_POS["NoixJEN"] = False
                RAMASSAGE_POS["NoixJES"] = False
            if Recuperer.nav_point == "NoixBWN" or Recuperer.nav_point == "NoixJWS":
                RAMASSAGE_POS["NoixBWN"] = False
                RAMASSAGE_POS["NoixBWS"] = False
            RAMASSAGE_POS[Recuperer.nav_point] = False
            robot.updateScore(0)

class Deposer(Action):
    name = "Deposer"
    nav_point = "NAN"


    @staticmethod
    def recup_point(_,cote):
        angle = DEPOT_ANG[Deposer.nav_point] if cote else DEPOT_ANG[Deposer.nav_point] + np.pi
        return (Deposer.nav_point,angle)
    
    @staticmethod
    def calcul_cote_couleur(robot):
        color_ours = Caisse.BLEU if robot.color == Team.BLEU else Caisse.JAUNE # Notre couleur de caisse
        color_notOurs = Caisse.BLEU if color_ours == Caisse.JAUNE else Caisse.BLEU # La color opposee
        if robot.cote_droit_ours() :
            cote,couleur = True,color_ours
        elif robot.cote_gauche_ours():
            cote,couleur = False,color_ours
        elif not robot.cote_droit_vide():
            cote,couleur = True,color_notOurs
        else :
            cote,couleur = False,color_notOurs
        return (cote,couleur)
    
    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        recup = py_trees.composites.Sequence("Deposer", True)

        print("===========Relache:",Deposer.calcul_cote_couleur(robot),"=============")

        recup.add_children([
            WaitSeconds(0.5),
            Navigate(lambda x : Deposer.recup_point(x,Deposer.calcul_cote_couleur(robot)[0])), 
            Relacher(Deposer.calcul_cote_couleur(robot))
        ])
        return recup
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        def mostRewardingDepotPoint():
                max_reward = -1
                max_wpt = "NAN"
                for wpt in DEPOT_POS.keys():
                    if DEPOT_POS[wpt]<2:
                        # On a des valeurs qui dependent de la distance, le gain minimal est 3 (a la distance max theorique) jusqu'a 6.
                        val = 6  - 3 * (robot.distance_from(wpt)/DISTANCE_MAX) #- SEUIL_AGRESSIVITE * (distance robot adverse/ distanceMax)
                    else :
                        val = 0
                    if max_reward < val:
                        max_reward = val
                        max_wpt = wpt
                return max_wpt,max_reward
        
        if (not robot.cote_droit_vide()) or (not robot.cote_gauche_vide()):
            if (robot.cote_droit_ours()) or (robot.cote_gauche_ours()):
                Deposer.nav_point, max_reward = mostRewardingDepotPoint()
                return max_reward
            else :
                if world.nid < 6 :
                    Deposer.nav_point, max_reward = START_POS[robot.color][robot.strat][0],6  - 3 * (robot.distance_from(START_POS[robot.color][robot.strat][0])/DISTANCE_MAX)
                    return max_reward
                else:
                    # On va tenter de retourner
                    return 0
        else:
            return 0
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        if status == py_trees.common.Status.SUCCESS:
            if Deposer.nav_point == START_POS[robot.color][robot.strat][0]:
                world.nid+=2
                robot.updateScore(4)
            else :
                DEPOT_POS[Deposer.nav_point] += 2
                robot.updateScore(6)

class Retourner(Action):
    name = "Retourner"
    nav_point = "NAN"

    @staticmethod
    def recup_point(_,cote):
        angle = DEPOT_ANG[Retourner.nav_point] if cote else DEPOT_ANG[Retourner.nav_point] + np.pi
        return (Retourner.nav_point,angle)
    
    @staticmethod
    def calcul_cote_couleur(robot):
        notre_couleur = Caisse.BLEU if robot.color == Team.BLEU else Caisse.JAUNE # Notre couleur de caisse
        pas_notre_couleur = Caisse.BLEU if notre_couleur == Caisse.JAUNE else Caisse.BLEU # La color opposee
        if robot.cote_droit_ours() :
            cote,couleur = True,notre_couleur
        elif robot.cote_gauche_ours():
            cote,couleur = False,notre_couleur
        elif not robot.cote_droit_vide():
            cote,couleur = True,pas_notre_couleur
        else :
            cote,couleur = False,pas_notre_couleur
        return (cote,couleur)
    
    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        recup = py_trees.composites.Sequence("Retourner", True)

        print("=========== Retourner:",Retourner.calcul_cote_couleur(robot)," =============")
        recup.add_children([
            WaitSeconds(0.5),
            Navigate(lambda x : Retourner.recup_point(x,Retourner.calcul_cote_couleur(robot)[0])), 
            Revolutionner(Retourner.calcul_cote_couleur(robot))
        ])
        return recup
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        def mostRewardingDepotPoint():
            max_reward = -1
            max_wpt = "NAN"
            for wpt in DEPOT_POS.keys():
                if DEPOT_POS[wpt]<2:
                    # On a des valeurs qui dependent de la distance, le gain minimal est 3 (a la distance max theorique) jusqu'a 6.
                    val = 6  - 3 * (robot.distance_from(wpt)/DISTANCE_MAX) #- SEUIL_AGRESSIVITE * (distance robot adverse/ distanceMax)
                else :
                    val = 0
                if max_reward < val:
                    max_reward = val
                    max_wpt = wpt
            return max_wpt,max_reward
        
        if (not robot.cote_droit_vide() and not robot.cote_droit_ours()) or (not robot.cote_gauche_vide() and not robot.cote_gauche_ours()):
            if world.nid >= 6:
                Retourner.nav_point, max_reward = mostRewardingDepotPoint()
                return max_reward
            else :
                return 0
        else:
            return 0
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        if status == py_trees.common.Status.SUCCESS:
            DEPOT_POS[Deposer.nav_point] += 2
            robot.updateScore(6)


#######################################################################################################################################################################################  2025


##########################
###  Action Banderole  ###
##########################

# class BanderoleAction(Action):
#     name = "Banderole"

#     @staticmethod
#     def create_bt(robot: Robot, world: World) -> Behaviour:
#         poserBanderolle = py_trees.composites.Sequence("Poser la banderolle", True)
#         poserBanderolle.add_children([
#             # GoTo zone banderole
#             WaitSeconds(0.5),
#             Bouge(Speed.from_dir(-120,100), 2),
#             LiftBanderole(False),
#             Bouge(Speed.from_dir(-120,-100), 3),
#             DeployMacon(),
#         ])
#         return poserBanderolle
    
#     @staticmethod
#     def reward(robot: Robot, world: World) -> float:
#         if world.thermo_positioned:
#             return 0
#         else:
#             return 20
    
#     @staticmethod
#     def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
#         if status == py_trees.common.Status.SUCCESS:
#             world.thermo_positioned = True
#             robot.updateScore(20)


##########################
###     Action End     ###
##########################

class EndAction(Action):
    name = "End"
    
    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        return EndStrat()
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        if world.time_left() > 0:
            # tiny reward to select this one if no  other action is possible
            return 1
        else:
            # crazy high reward to be sure to be selected
            return 1e6


##########################
### Action Match Start ###
##########################

class MatchStartAction(Action):
    name = "MatchStart"

    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        match_start = py_trees.composites.Sequence("GARDE A VOUS !", True)
        match_start.add_children([
            MoveBrasD(PosTentacle.HAUT),
            MoveBrasG(PosTentacle.HAUT),
            WaitMatchStart(),
            MoveBrasD(PosTentacle.HAUT),
            MoveBrasG(PosTentacle.HAUT)
        ])
        return match_start
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        # match not started yet
        if not world.match_started():
            return 1e6
        else:
            return 0

    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        if status == py_trees.common.Status.SUCCESS:
            print("GOOD Started")
            #subprocess.Popen(["ecal_rec","-r","110","--activate"])


##########################
###   Action Go Home   ###
##########################

class GoHomeAction(Action):
    name = "GoHome"

    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        nav_pt = END_POS[robot.color][robot.strat]
        def nana(_):
            return nav_pt
        terminate = py_trees.composites.Sequence("Go to the Nest", True)
        terminate.add_children([
            Navigate(nana),
            WaitUntil(94,world.matchStartTime),
            Move(robot,200,0,Velocity.NORMAL.value) 
        ])
            #Bouge(Speed(200,0,0),4)])
        return terminate
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        if not world.match_started() or world.backInZone:
            # match not started or already back home
            return 0
        
        _nav_pt = END_POS[robot.color][robot.strat]
        estimated_time = robot.pos.distance(robot.dest_to_pos(_nav_pt)) / Velocity.NORMAL.value.xy_norm() + 5
        if (world.time_left()-15) < estimated_time:
            # rush to home, high reward
            return 1000
        else:
            return 2
    
    @staticmethod
    def start_cb(robot: Robot, world: World) -> None:
        robot.set_speed(Velocity.NORMAL.value)
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        if status == py_trees.common.Status.SUCCESS:
            world.backInZone = True
            robot.updateScore(10)

##########################
###   Action PoussePousse   ###
##########################

# class PoussePousse(Action):
#     name = "poussepousse"

#     @staticmethod
#     def create_bt(robot: Robot, world: World) -> Behaviour:
#         nav_pt = INTEREST[robot.color][robot.strat]
#         def nana(_):
#             return nav_pt
#         simple_pousse = py_trees.composites.Sequence("poussepousse", True)
#         simple_pousse.add_children([
#             Navigate(nana),
#             Bouge(Speed(100,0,0), 5),
#             Bouge(Speed(-100,0,0), 2),
#         ])
#         return simple_pousse
    
#     @staticmethod
#     def reward(robot: Robot, world: World) -> float:
#         # gradin = INTEREST[robot.color][robot.strat][0]
#         if world.gradin_pousse_pousse:
#             return 0
#         else:
#             return 5
#     @staticmethod
#     def start_cb(robot: Robot, world: World) -> None:
#         robot.locomotion.select_velocity(Velocity.NORMAL)
#         robot.actionneurs.moveRentreur(INSIDE)

#     @staticmethod
#     def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
#         if status == py_trees.common.Status.SUCCESS:
#             # gradin = INTEREST[robot.color][robot.strat][0]
#             world.gradin_pousse_pousse = True
#             robot.updateScore(4)


# class Gradin(Action):
#     name = "gradin"

#     @staticmethod
#     def create_bt(robot: Robot, world: World) -> Behaviour:
#         first = INTEREST[robot.color][robot.strat][0]
#         def First(_):
#             return first
#         second = INTEREST[robot.color][robot.strat][1]
#         def Second(_):
#             return second
#         gradin = py_trees.composites.Sequence("gradin", True)
#         gradin.add_children([
#             Navigate(First),
#             DeployMacon(),
#             WaitSeconds(0.5),
#             Bouge(Speed(150,0,-0.1),2),
#             GrabLowConserve(True),
#             LiftPlanche(UP),
#             Bouge(Speed(0,0,0.5),1),
#             Bouge(Speed(0,0,-0.5),2),
#             WaitSeconds(0.5),
#             LiftConserve(ValeurActionneur.AscenseurAimantUP),
#             WaitSeconds(1),
#             MoveRentreur(OUTSIDE),
#             WaitSeconds(0.1),
#             GrabHighConserve(True),
#             WaitSeconds(0.5),
#             GrabLowConserve(False),
#             WaitSeconds(1),
#             LockPlanche(True),
#             LiftConserve(ValeurActionneur.AscenseurAimantDOWN),
#             WaitSeconds(1),
#             MoveRentreur(PUSH),
#             WaitSeconds(0.5),
#             GrabLowConserve(True),
#             WaitSeconds(1),
#             LiftConserve(ValeurActionneur.AscenseurAimantINTERMEDIAIRE),
#             WaitSeconds(1),
#             LiftConserve(ValeurActionneur.AscenseurAimantDOWN),
#             Navigate(Second),
#             Bouge(Speed(100,0,0), 5),
#             GrabHighConserve(False),
#             GrabLowConserve(False),
#             Bouge(Speed(-200,0,0),0.5),
#             LiftPlanche(MID),
#             WaitSeconds(1),
#             #LockPlanche(False),
#             PinceFinal(True),
#             WaitSeconds(0.6),
#             #PinceFinal(True),
#             LockPlanche(False),
#             Bouge(Speed(-150,0,0),2),

#             ])
#         return gradin
    
#     @staticmethod
#     def reward(robot: Robot, world: World) -> float:
        
#         if not world.match_started():
#             return 0
#         else:
#             gradin = INTEREST[robot.color][robot.strat][0][0]
#             if not world.Gradin[gradin]:
#                 return 10
#             else:
#                 return 0

#     @staticmethod
#     def start_cb(robot: Robot, world: World) -> None:
#         robot.locomotion.select_velocity(Velocity.NORMAL)

#     @staticmethod
#     def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
#         if status == py_trees.common.Status.SUCCESS:
#             gradin = INTEREST[robot.color][robot.strat][0][0]
#             world.Gradin[gradin] = True
#             robot.updateScore(12)
