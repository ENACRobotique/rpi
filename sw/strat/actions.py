from planner import Action
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.composites import Selector, Sequence
import sys
import numpy as np
sys.path.append("../..")
from robot import Robot, COTE_DROIT, COTE_GAUCHE, Velocity
from common import Speed
from world import POINT_DEPOT, POINT_RAMASSAGE, World
from bt_essentials import CAISSE0_POS, DEPOT0_POS, MatchTimer, Navigate, WaitMatchStart, WaitUntil
from bt_essentials import EndStrat, END_POS, WaitSeconds, THERMO_POS, MoveTo, Move, MoveSpeed, START_POS, CAISSETHERMO_POS, DEPOT1_POS, CAISSE1_POS,DEPOT2_POS, DEPOT3_POS, DEPOT4_POS, CAISSE2_POS
from typing import Callable
from dataclasses import dataclass
import time
import subprocess
from IO.IO_BT import *

DISTANCE_MAX  = 3000*2 + 1500*2 #np.sqrt(13) * 1000 # en mm, sqrt(2**2 + 3**2)
SEUIL_AGRESSIVITE = 3 # Compris entre 4 (Ghandi) et 2 (Chabal)

##################################
###      Action Thermometre    ###
##################################

class ThermometreAction(Action):
    name = "Thermometre"

    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        cote = False if robot.color == Team.JAUNE else True # ie on recup cote Gauche avec le jaune pour avoir bras droit libre (et inversement cote bleu)
        bougerThermo = py_trees.composites.Sequence("Thermometre", True)
        bougerThermo.add_children([
            WaitSeconds(0.3),

            #Thermo Action
            MoveTo(robot.dest_to_pos(CAISSETHERMO_POS[robot.color][robot.strat])),
            #WaitSeconds(1),
            Aligner(cote),
            Attraper(cote),
            MoveTo(robot.dest_to_pos(THERMO_POS[robot.color][robot.strat])),
            MoveSpeed(Speed(-200,0,0),1),
            MoveBrasThermo(PosTentacle.THERMO),
            Move(500,0),
            MoveBrasThermo(PosTentacle.HAUT),
        ])
        return bougerThermo
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        if world.thermo_positioned:
            return 0
        else:
            return 10
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        POINT_RAMASSAGE[CAISSETHERMO_POS[robot.color][robot.strat][0]].ramasse = 0 # On dit qu'on a recup la caisse
        world.thermo_positioned = True
        if status == py_trees.common.Status.SUCCESS:
            robot.updateScore(10)


class RecupererDroite(Action):
    name = "RecupererDroite"
    nav_point = "NAN"

    @staticmethod
    def recup_point(_):
        angle = POINT_RAMASSAGE[RecupererDroite.nav_point].angle
        return (RecupererDroite.nav_point,normalize_angle(angle))
    
    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        recup = py_trees.composites.Sequence("Recuperer", True)
        cote = True
        recup.add_children([
            WaitSeconds(0.2),
            Navigate(lambda x : RecupererDroite.recup_point(x)), 
            Aligner(cote),
            Attraper(cote)
        ])
        return recup
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        if robot.cote_droit_vide():
            # fonction qui calcule le point de ramasse qui rapporte le plus
            def mostRewardingGrabPoint():
                max_reward = -1
                max_wpt = "NAN"
                for wpt in POINT_RAMASSAGE.keys():
                    if POINT_RAMASSAGE[wpt].ramasse>0:
                        # On a des valeurs qui dependent de la distance, le gain minimal est 3 (a la distance max theorique) jusqu'a 6.
                        try :
                            val = POINT_RAMASSAGE[wpt].ramasse * (6  - 3 * (robot.distance_du_path(wpt)/DISTANCE_MAX) - 0.5 * abs(normalize_angle(POINT_RAMASSAGE[wpt].angle-robot.pos.theta)/np.pi)) #- SEUIL_AGRESSIVITE * (distance robot adverse/ distanceMax)
                        except KeyError :
                            val = - 10
                    else :
                        val = 0
                    if max_reward < val:
                        max_reward = val
                        max_wpt = wpt
                return max_wpt,max_reward
        
            RecupererDroite.nav_point, max_reward = mostRewardingGrabPoint()
            return max_reward
        else:
            return 0
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        if RecupererDroite.nav_point == "NoixJEN" or RecupererDroite.nav_point == "NoixJES":
            POINT_RAMASSAGE["NoixJEN"].ramasse = 0
            POINT_RAMASSAGE["NoixJES"].ramasse = 0
        if RecupererDroite.nav_point == "NoixBWN" or RecupererDroite.nav_point == "NoixJWS":
            POINT_RAMASSAGE["NoixBWN"].ramasse = 0
            POINT_RAMASSAGE["NoixBWS"].ramasse = 0
        POINT_RAMASSAGE[RecupererDroite.nav_point].ramasse = 0

class RecupererGauche(Action):
    name = "Recuperer"
    nav_point = "NAN"

    @staticmethod
    def recup_point(_):
        angle = POINT_RAMASSAGE[RecupererGauche.nav_point].angle + np.pi
        return (RecupererGauche.nav_point,normalize_angle(angle))
    
    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        recup = py_trees.composites.Sequence("RecupererGauche", True)
        cote = False
        recup.add_children([
            WaitSeconds(0.2),
            Navigate(lambda x : RecupererGauche.recup_point(x)), 
            Aligner(cote),
            Attraper(cote)
        ])
        return recup
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        if robot.cote_gauche_vide():
            # fonction qui calcule le point de ramasse qui rapporte le plus
            def mostRewardingGrabPoint():
                max_reward = -1
                max_wpt = "NAN"
                for wpt in POINT_RAMASSAGE.keys():
                    if POINT_RAMASSAGE[wpt].ramasse>0:
                        # On a des valeurs qui dependent de la distance, le gain minimal est 3 (a la distance max theorique) jusqu'a 6.
                        try :
                            val = POINT_RAMASSAGE[wpt].ramasse * (6  - 3 * (robot.distance_du_path(wpt)/DISTANCE_MAX) - 0.5 * abs(normalize_angle(POINT_RAMASSAGE[wpt].angle - robot.pos.theta + np.pi)/np.pi)) #- SEUIL_AGRESSIVITE * (distance robot adverse/ distanceMax)
                        except KeyError :
                            val = - 10
                    else :
                        val = 0
                    if max_reward < val:
                        max_reward = val
                        max_wpt = wpt
                return max_wpt,max_reward
        
            RecupererGauche.nav_point, max_reward = mostRewardingGrabPoint()
            return max_reward
        else:
            return 0
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        if RecupererGauche.nav_point == "NoixJEN" or RecupererGauche.nav_point == "NoixJES":
            POINT_RAMASSAGE["NoixJEN"].ramasse = 0
            POINT_RAMASSAGE["NoixJES"].ramasse = 0
        if RecupererGauche.nav_point == "NoixBWN" or RecupererGauche.nav_point == "NoixJWS":
            POINT_RAMASSAGE["NoixBWN"].ramasse = 0
            POINT_RAMASSAGE["NoixBWS"].ramasse = 0
        POINT_RAMASSAGE[RecupererGauche.nav_point].ramasse = 0

class DeposerDroite(Action):
    name = "DeposerDroite"
    nav_point = "NAN"


    @staticmethod
    def recup_point(_):
        angle = POINT_DEPOT[DeposerDroite.nav_point].angle
        return (DeposerDroite.nav_point,normalize_angle(angle))
    
    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        recup = py_trees.composites.Sequence("DeposerDroite", True)
        notre_couleur = Caisse.BLEU if robot.color == Team.BLEU else Caisse.JAUNE # Notre couleur de caisse

        print(f"=========== Relache: Cote droit et {notre_couleur}=============")

        recup.add_children([
            WaitSeconds(0.2),
            Navigate(lambda x : DeposerDroite.recup_point(x)), 
            Relacher((True,notre_couleur))
        ])
        return recup
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        def mostRewardingDepotPoint():
                max_reward = -1
                max_wpt = "NAN"
                for wpt in POINT_DEPOT.keys():
                    if len(POINT_DEPOT[wpt].contient)<2:
                        # On a des valeurs qui dependent de la distance, le gain minimal est 3 (a la distance max theorique) jusqu'a 6.
                        try :
                            val = 6  - 3 * (robot.distance_du_path(wpt)/DISTANCE_MAX) - 0.5 * abs(normalize_angle(POINT_DEPOT[wpt].angle - robot.pos.theta)/np.pi)#- SEUIL_AGRESSIVITE * (self.robot.distance() / distanceMax)
                        except KeyError :
                            val = - 10
                    else :
                        val = 0
                    if max_reward < val:
                        max_reward = val
                        max_wpt = wpt
                return max_wpt,max_reward
        
        if robot.cote_droit_ours():
            DeposerDroite.nav_point, max_reward = mostRewardingDepotPoint()
            return max_reward
        else :
            return 0
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        if status == py_trees.common.Status.SUCCESS:
            if DeposerDroite.nav_point == START_POS[robot.color][robot.strat][0]:
                world.nid+=2
                robot.updateScore(4)
            else :
                notre_couleur = Caisse.BLEU if robot.color == Team.BLEU else Caisse.JAUNE # Notre couleur de caisse
                if DeposerDroite.nav_point == "FrigoJES":
                    POINT_DEPOT["FrigoJEN"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoJEN"].contient.append(notre_couleur)
                if DeposerDroite.nav_point == "FrigoJEN":
                    POINT_DEPOT["FrigoJES"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoJES"].contient.append(notre_couleur)
                        
                if DeposerDroite.nav_point == "FrigoBWS":
                    POINT_DEPOT["FrigoBWN"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoBWN"].contient.append(notre_couleur)
                if DeposerDroite.nav_point == "FrigoBWN":
                    POINT_DEPOT["FrigoBWS"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoBWS"].contient.append(notre_couleur)

                if DeposerDroite.nav_point == "FrigoMidNS":
                    POINT_DEPOT["FrigoMidNN"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoMidNN"].contient.append(notre_couleur)
                if DeposerDroite.nav_point == "FrigoMidNN":
                    POINT_DEPOT["FrigoMidNS"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoMidNS"].contient.append(notre_couleur)

                POINT_DEPOT[DeposerDroite.nav_point].contient.append(notre_couleur)
                POINT_DEPOT[DeposerDroite.nav_point].contient.append(notre_couleur)
                robot.updateScore(6)

class DeposerGauche(Action):
    name = "DeposerGauche"
    nav_point = "NAN"


    @staticmethod
    def recup_point(_):
        angle = POINT_DEPOT[DeposerGauche.nav_point].angle + np.pi
        return (DeposerGauche.nav_point,normalize_angle(angle))
    
    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        recup = py_trees.composites.Sequence("DeposerGauche", True)

        notre_couleur = Caisse.BLEU if robot.color == Team.BLEU else Caisse.JAUNE # Notre couleur de caisse

        print(f"===========Relache: cote gauche et {notre_couleur}=============")

        recup.add_children([
            WaitSeconds(0.2),
            Navigate(lambda x : DeposerGauche.recup_point(x)), 
            Relacher((False,notre_couleur))
        ])
        return recup
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        def mostRewardingDepotPoint():
                max_reward = -1
                max_wpt = "NAN"
                for wpt in POINT_DEPOT.keys():
                    if len(POINT_DEPOT[wpt].contient)<2:
                        # On a des valeurs qui dependent de la distance, le gain minimal est 3 (a la distance max theorique) jusqu'a 6.
                        try :
                            val = 6  - 3 * (robot.distance_du_path(wpt)/DISTANCE_MAX) - 0.5 * abs(normalize_angle(POINT_DEPOT[wpt].angle - robot.pos.theta + np.pi)/np.pi) #- SEUIL_AGRESSIVITE * (self.robot.distance() / distanceMax)
                        except KeyError :
                            val = - 10
                    else :
                        val = 0
                    if max_reward < val:
                        max_reward = val
                        max_wpt = wpt
                return max_wpt,max_reward
        
        if robot.cote_gauche_ours():
            DeposerGauche.nav_point, max_reward = mostRewardingDepotPoint()
            return max_reward
        else :
            return 0
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        if status == py_trees.common.Status.SUCCESS:
            if DeposerGauche.nav_point == START_POS[robot.color][robot.strat][0]:
                world.nid+=2
                robot.updateScore(4)
            else :
                notre_couleur = Caisse.BLEU if robot.color == Team.BLEU else Caisse.JAUNE # Notre couleur de caisse
                if DeposerGauche.nav_point == "FrigoJES":
                    POINT_DEPOT["FrigoJEN"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoJEN"].contient.append(notre_couleur)
                if DeposerGauche.nav_point == "FrigoJEN":
                    POINT_DEPOT["FrigoJES"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoJES"].contient.append(notre_couleur)
                        
                if DeposerGauche.nav_point == "FrigoBWS":
                    POINT_DEPOT["FrigoBWN"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoBWN"].contient.append(notre_couleur)
                if DeposerGauche.nav_point == "FrigoBWN":
                    POINT_DEPOT["FrigoBWS"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoBWS"].contient.append(notre_couleur)

                if DeposerGauche.nav_point == "FrigoMidNS":
                    POINT_DEPOT["FrigoMidNN"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoMidNN"].contient.append(notre_couleur)
                if DeposerGauche.nav_point == "FrigoMidNN":
                    POINT_DEPOT["FrigoMidNS"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoMidNS"].contient.append(notre_couleur)

                POINT_DEPOT[DeposerGauche.nav_point].contient.append(notre_couleur)
                POINT_DEPOT[DeposerGauche.nav_point].contient.append(notre_couleur)
                robot.updateScore(6)

class RetournerDroite(Action):
    name = "Retourner"
    nav_point = "NAN"

    @staticmethod
    def recup_point(_):
        angle = POINT_DEPOT[RetournerDroite.nav_point].angle
        return (RetournerDroite.nav_point,normalize_angle(angle))
    
    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        recup = py_trees.composites.Sequence("Retourner", True)

        notre_couleur = Caisse.BLEU if robot.color == Team.BLEU else Caisse.JAUNE # Notre couleur de caisse
        pas_notre_couleur = Caisse.BLEU if notre_couleur == Caisse.JAUNE else Caisse.BLEU # La color opposee


        print(f"=========== Retourner: cote Droit {pas_notre_couleur} =============")
        recup.add_children([
            WaitSeconds(0.2),
            Navigate(lambda x : RetournerDroite.recup_point(x)), 
            Revolutionner((True,pas_notre_couleur))
        ])
        return recup
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        def mostRewardingDepotPoint():
            max_reward = -1
            max_wpt = "NAN"
            for wpt in POINT_DEPOT.keys():
                if len(POINT_DEPOT[wpt].contient)<2:
                    # On a des valeurs qui dependent de la distance, le gain minimal est 3 (a la distance max theorique) jusqu'a 6.
                    try :
                        val = 6  - 3 * (robot.distance_du_path(wpt)/DISTANCE_MAX) - 0.5 * abs(normalize_angle(POINT_DEPOT[wpt].angle - robot.pos.theta)/np.pi)#- SEUIL_AGRESSIVITE * (distance robot adverse/ distanceMax)
                    except KeyError :
                        val = - 10
                else :
                    val = 0
                if max_reward < val:
                    max_reward = val
                    max_wpt = wpt
            return max_wpt,max_reward
        
        if not robot.cote_droit_vide() and not robot.cote_droit_ours():
            RetournerDroite.nav_point, max_reward = mostRewardingDepotPoint()
            return max_reward
        else:
            return 0
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        if status == py_trees.common.Status.SUCCESS:
                notre_couleur = Caisse.BLEU if robot.color == Team.BLEU else Caisse.JAUNE # Notre couleur de caisse
                if RetournerDroite.nav_point == "FrigoJES":
                    POINT_DEPOT["FrigoJEN"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoJEN"].contient.append(notre_couleur)
                if RetournerDroite.nav_point == "FrigoJEN":
                    POINT_DEPOT["FrigoJES"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoJES"].contient.append(notre_couleur)
                        
                if RetournerDroite.nav_point == "FrigoBWS":
                    POINT_DEPOT["FrigoBWN"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoBWN"].contient.append(notre_couleur)
                if RetournerDroite.nav_point == "FrigoBWN":
                    POINT_DEPOT["FrigoBWS"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoBWS"].contient.append(notre_couleur)

                if RetournerDroite.nav_point == "FrigoMidNS":
                    POINT_DEPOT["FrigoMidNN"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoMidNN"].contient.append(notre_couleur)
                if RetournerDroite.nav_point == "FrigoMidNN":
                    POINT_DEPOT["FrigoMidNS"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoMidNS"].contient.append(notre_couleur)

                POINT_DEPOT[RetournerDroite.nav_point].contient.append(notre_couleur)
                POINT_DEPOT[RetournerDroite.nav_point].contient.append(notre_couleur)
                robot.updateScore(6)

class RetournerGauche(Action):
    name = "Retourner"
    nav_point = "NAN"

    @staticmethod
    def recup_point(_):
        angle = POINT_DEPOT[RetournerGauche.nav_point].angle + np.pi
        return (RetournerGauche.nav_point,normalize_angle(angle))
    
    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        recup = py_trees.composites.Sequence("Retourner", True)

        notre_couleur = Caisse.BLEU if robot.color == Team.BLEU else Caisse.JAUNE # Notre couleur de caisse
        pas_notre_couleur = Caisse.BLEU if notre_couleur == Caisse.JAUNE else Caisse.BLEU # La color opposee


        print(f"=========== Retourner: cote Gauche {pas_notre_couleur} =============")
        recup.add_children([
            WaitSeconds(0.2),
            Navigate(lambda x : RetournerGauche.recup_point(x)), 
            Revolutionner((False,pas_notre_couleur))
        ])
        return recup
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        def mostRewardingDepotPoint():
            max_reward = -1
            max_wpt = "NAN"
            for wpt in POINT_DEPOT.keys():
                if len(POINT_DEPOT[wpt].contient)<2:
                    # On a des valeurs qui dependent de la distance, le gain minimal est 3 (a la distance max theorique) jusqu'a 6.
                    try :
                        val = 6  - 3 * (robot.distance_du_path(wpt)/DISTANCE_MAX) - 0.5 * abs(normalize_angle(POINT_DEPOT[wpt].angle - robot.pos.theta + np.pi)/np.pi)#- SEUIL_AGRESSIVITE * (distance robot adverse/ distanceMax)
                    except KeyError :
                        val = - 10
                else :
                    val = 0
                if max_reward < val:
                    max_reward = val
                    max_wpt = wpt
            return max_wpt,max_reward
        
        if not robot.cote_gauche_vide() and not robot.cote_gauche_ours():
            RetournerGauche.nav_point, max_reward = mostRewardingDepotPoint()
            return max_reward
        else:
            return 0
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        if status == py_trees.common.Status.SUCCESS:
                notre_couleur = Caisse.BLEU if robot.color == Team.BLEU else Caisse.JAUNE # Notre couleur de caisse
                if RetournerDroite.nav_point == "FrigoJES":
                    POINT_DEPOT["FrigoJEN"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoJEN"].contient.append(notre_couleur)
                if RetournerDroite.nav_point == "FrigoJEN":
                    POINT_DEPOT["FrigoJES"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoJES"].contient.append(notre_couleur)
                        
                if RetournerDroite.nav_point == "FrigoBWS":
                    POINT_DEPOT["FrigoBWN"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoBWN"].contient.append(notre_couleur)
                if RetournerDroite.nav_point == "FrigoBWN":
                    POINT_DEPOT["FrigoBWS"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoBWS"].contient.append(notre_couleur)

                if RetournerDroite.nav_point == "FrigoMidNS":
                    POINT_DEPOT["FrigoMidNN"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoMidNN"].contient.append(notre_couleur)
                if RetournerDroite.nav_point == "FrigoMidNN":
                    POINT_DEPOT["FrigoMidNS"].contient.append(notre_couleur)
                    POINT_DEPOT["FrigoMidNS"].contient.append(notre_couleur)

                POINT_DEPOT[RetournerDroite.nav_point].contient.append(notre_couleur)
                POINT_DEPOT[RetournerDroite.nav_point].contient.append(notre_couleur)
                robot.updateScore(6)

#########################################
###### MATCH BASIQUE ####################
#########################################

class Match(Action):
    name = "Match"
    
    @staticmethod
    def create_bt(robot: Robot, world: World) -> Behaviour:
        match = py_trees.composites.Sequence("Match", True)

        coteThermo = False if robot.color == Team.JAUNE else True # ie on recup cote Gauche avec le jaune pour avoir bras droit libre (et inversement cote bleu)

        notre_couleur = Caisse.BLEU if robot.color == Team.BLEU else Caisse.JAUNE # Notre couleur de caisse
        pas_notre_couleur = Caisse.BLEU if notre_couleur == Caisse.JAUNE else Caisse.JAUNE # La color opposee

        match.add_children([

            WaitSeconds(0.5),

            Navigate(lambda _: CAISSE0_POS[robot.color][robot.strat]), #MoveTo(robot.dest_to_pos(CAISSE0_POS[robot.color][robot.strat])),
            Aligner(coteThermo),
            Attraper(coteThermo,notre_couleur),
            Navigate(lambda _: DEPOT0_POS[robot.color][robot.strat]), #MoveTo(robot.dest_to_pos(DEPOT0_POS[robot.color][robot.strat])),
            Relacher((coteThermo, notre_couleur)),

            #Thermo Action
            Navigate(lambda _: CAISSETHERMO_POS[robot.color][robot.strat]),#MoveTo(robot.dest_to_pos(CAISSETHERMO_POS[robot.color][robot.strat])),
            #WaitSeconds(5),
            Aligner(coteThermo),
            Attraper(coteThermo),
            Navigate(lambda _: THERMO_POS[robot.color][robot.strat]), #MoveTo(robot.dest_to_pos(THERMO_POS[robot.color][robot.strat])),
            MoveSpeed(Speed(-200,0,0),1),
            MoveBrasThermo(PosTentacle.THERMO),
            Move(500,0),
            MoveBrasThermo(PosTentacle.HAUT),

            #### Retourner au depot 1
            Navigate(lambda _: DEPOT1_POS[robot.color][robot.strat]), #MoveTo(robot.dest_to_pos(DEPOT1_POS[robot.color][robot.strat])), 
            Revolutionner((coteThermo,pas_notre_couleur)),

            #### Recup CAISSE 1
            Move(0,np.pi),
            Navigate(lambda _: CAISSE1_POS[robot.color][robot.strat]), #MoveTo(robot.dest_to_pos(CAISSE1_POS[robot.color][robot.strat])),
            Aligner(not coteThermo),
            Attraper(not coteThermo),

            ### Deposer au depot 2
            Navigate(lambda _: DEPOT2_POS[robot.color][robot.strat]), #MoveTo(robot.dest_to_pos(DEPOT2_POS[robot.color][robot.strat])),
            Relacher((not coteThermo, notre_couleur)),

            ## Deposer au depot 3
            Navigate(lambda _: DEPOT3_POS[robot.color][robot.strat]), #MoveTo(robot.dest_to_pos(DEPOT3_POS[robot.color][robot.strat])),
            Relacher((coteThermo, notre_couleur)),

            #### Recup CAISSE 2
            Navigate(lambda _: CAISSE2_POS[robot.color][robot.strat]), #MoveTo(robot.dest_to_pos(CAISSE2_POS[robot.color][robot.strat])),
            Aligner(coteThermo),
            Attraper(coteThermo),

            ## Deposer au depot 4
            Navigate(lambda _: DEPOT4_POS[robot.color][robot.strat]), #MoveTo(robot.dest_to_pos(DEPOT4_POS[robot.color][robot.strat])),
            Relacher((coteThermo, notre_couleur)),

            ##
            Navigate(lambda _: DEPOT0_POS[robot.color][robot.strat]), #MoveTo(robot.dest_to_pos(DEPOT0_POS[robot.color][robot.strat])),
            Revolutionner((coteThermo, pas_notre_couleur)),

            Move(-200,np.pi),

            MoveBrasD(PosTentacle.BAS) if coteThermo == False else MoveBrasG(PosTentacle.BAS),

            Move(-800,0),

            Relacher((not coteThermo, Caisse.TOUT)),

            WaitUntil(94,world.matchStartTime)
            
        ])
        return match
    
    @staticmethod
    def reward(robot: Robot, world: World) -> float:
        if robot.strat == Strat.Audacieuse : return 0
        if world.main_match_action_done :
            return 0
        else :
            return 50
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        print("World time left : ",world.time_left())
        world.main_match_action_done = True
        return



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
            Relacher((False,Caisse.TOUT)),
            Relacher((True,Caisse.TOUT))
        ])
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
        pass
    
    @staticmethod
    def end_cb(robot: Robot, world: World, status: py_trees.common.Status) -> None:
        if status == py_trees.common.Status.SUCCESS:
            print("durée match restante :",world.MATCH_DURATION - (time.time() - world.matchStartTime))
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