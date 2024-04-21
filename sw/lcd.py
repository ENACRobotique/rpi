from enum import Enum
from abc import ABC, abstractmethod
from time import sleep

import messages_pb2 as llpb
import robot_state_pb2 as hgpb

import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber

class Team(Enum):
    BLEU = 1
    JAUNE = 2


class Etat(Enum):
    EN_MARCHE = 1
    REDEMARRE =  2
    ARRET = 3


class PositionMenu(Enum):
    """ sert à définir l'utilité du "ok" reçu"""
    MAIN_MENU = 1
    MENU = 2
    Page =  3


class Menu(ABC):
    """
    Classe abstraite dont héritera les différents menus du lcd
    """
    def __init__(self, title):
        self.title = title
        self.page : int
        self.pageMax : int
        self.options : list
    
    @abstractmethod
    def returnSelected(self):
        pass

    @abstractmethod
    def turnPage(self):
        pass


class MainMenu(Menu):
    def __init__(self, title):
        super().__init__(title="MenuStrat")
        self.selectedOption = ""  # initialisation de selectedOption

    def initialisation(self):
        """Initialise la stratb"""
        self.options = ["strat", "etat systemes"]  # initialisation des options
        self.page = 1
        self.selectedOption = self.options[self.page-1]
        self.pageMax = len(self.options)

    def returnSelected(self):
        return self.selectedOption
    
    def turnPage(self):
        self.page = (self.page + 1) % self.pageMax  # utilisation de % pour boucler sur les options
        self.selectedOption = self.options[self.page-1]


class MenuStrat(Menu):
    def __init__(self, title):
        super().__init__(title="MenuStrat")
        self.options = []  # initialisation de la liste des options
        self.selectedOption = ""


    def initialisation(self):
        """Initialise la strat"""
        self.options = ["nominal", "secours", "finale"]  # initialisation des options
        self.page = 1
        self.selectedOption = self.options[self.page-1]
        self.pageMax = len(self.options)


    def returnSelected(self):
        return self.selectedOption
    

    def turnPage(self):
        """tourne la page et revient à la page 1 après avoir fait le tour de toutes"""
        self.page = (self.page + 1) % self.pageMax  # utilisation de % pour boucler sur les options
        self.selectedOption = self.options[self.page-1]


class LCD(object):  
    """
    Architecture : 
    Main menu : choix des menus ( 1seul pour l'instant). Quand on clique OK, on rentre dans l'un des sous menus
    Sous menus : propose plusieurs choix
    """
    def __init__(self):
        ### attributs 
        self.team = Team.BLEU
        self.menu = None
        self.score = 0
        self.etatLidar = False
        self.tirette = False # True, tirée, False, encore en marche
        self.Mainmenu = MainMenu("MainMenu")
        self.stratMenu = MenuStrat("StratMenu")
        self.posMenu = PositionMenu.MAIN_MENU  # où on se situe dans l'architecture : menu ou sous menu
        self.stratChoisie = False

        ###publisher et subscribers
        self.team_sub = ProtoSubscriber("team", hgpb.Team)
        self.team_sub.set_callback(self.onReceivedTeam)

        self.action_sub = ProtoSubscriber("action", hgpb.Action) # ok ou return
        self.action_sub.set_callback(self.onReceivedAction)

        self.score_sub = ProtoSubscriber("set_score", hgpb.Match)
        self.score_sub.set_callback(self.onReceivedScore)

        self.color_pub = ProtoSubscriber("color", hgpb.Color)


    def initialisation(self):
        """Initialise le lcd"""
        self.menu = self.Mainmenu
        self.posMenu = PositionMenu.MAIN_MENU
        self.Mainmenu.initialisation()
        self.stratMenu.initialisation()


    def onReceivedScore(self, topic_name, hlm, timelf):
        self.score = hlm.score
        self.send_msg(self.score)


    def onReceivedAction(self, topic_name, hlm, timelf):
        if (hlm.ok == 1) and (hlm.ret ==0):
            self.onReceivedOk()

        elif (hlm.ret == 1) and (hlm.ok ==0):
            self.onReceivedReturn()

        elif hlm.turn ==1:
            self.onReceivedTurn


    def onReceivedTeam(self):
        """
        Lorsqu'on appuie sur le bouton la 1ere fois, on est bleu, si on rappuie on change de couleur
        On commence en None, comme ça le lcd alerte si on veut commencer le match en None
        """
        if self.team == Team.BLEU:
            self.team = Team.JAUNE
            self.send_color( 255, 255, 0)
        elif self.team == Team.JAUNE:
            self.team = Team.BLEU
            self.send_color( 0, 0, 255)
        else : 
            self.team = Team.BLEU
            self.send_color( 0, 0, 255)


    def send_msg(self,msg):
        """
        fonction qui envoie les données à la com 
        Sera sûrement inutile par la suite
        """
        print(msg)


    def onReceivedTurn(self):
        """
        fonction qui permet de tourner la page d'un menu. On considère que le bas niveau décide d'envoyer 1 turn page tous les x crans de potard
        """
        self.menu.turnPage()


    def onReceivedReturn(self):
        """
        Pour l'instant il n'y a pas de sousous menus, donc si on fait return, soit ça ne fait rien, soit ça nous renvoie au menu principal
        à modifier si on rajoute des sousousmenus
        """
        if self.posMenu != PositionMenu.MAIN_MENU:
            # On revient au menu principal 
            self.posMenu = PositionMenu.MAIN_MENU


    def onReceivedOk(self):
        """
        On plonge dans le sous menu, ou, on valide le choix proposé
        Si le choix est validé, on remonte au main
        """
        if self.posMenu == PositionMenu.MAIN_MENU:
            #Si on est au menu principal, le menu en cours devient le menu affiché à la page validée
            menu_selectionne = self.menu.returnSelected()

            if isinstance(menu_selectionne, str) and menu_selectionne == "strat": # on rentre dans le menu choix de strat
                self.menu = self.stratMenu
            ### Rajouter en elif les autres menus

            self.posMenu = PositionMenu.MENU
            
        elif self.posMenu == PositionMenu.MENU:
            # Dans ce cas, on revient au menu principal, et on retient l'info selectionnée 
            info_selectionne = self.menu.returnSelected()
            self.send_msg(info_selectionne)  # on envoi l'info et sera traitée plus tard 
            if self.menu == self.stratMenu:
                self.stratChoisie = True
            self.posMenu = PositionMenu.MAIN_MENU


    def configOk(self):
        """verifie si tout est prêt à être lancé, clignotte rouge sinon"""
        if ((self.stratChoisie == False) ): ## verifier sur ecal si topic odompos publie qqchose et IO state 
            self.send_msg("configNok")
        else:
            self.send_msg("ConfigOk")


    def send_color(self, R:int, G:int, B:int):
        color_msg = hgpb.Color(r=R,g=G,b=B)
        self.color_pub.send(color_msg)



### TODO 
# Communication avec le bas niveau, qui va recevoir les messages, bridge_interface
# communication robot pour les infos importantes
# send score 
# d'autres à décider plus tard


if __name__ == "__main__": 
    ### test , simu  #plus d'actualité avec e-cal
    lcd = LCD()
    lcd.initialisation()

    lcd.configOk()
    sleep(1)

    lcd.onReceivedOk()
    lcd.onReceivedTurn()
    sleep(0.5)
    lcd.onReceivedTurn()
    sleep(0.5)
    lcd.onReceivedOk()
    lcd.configOk()