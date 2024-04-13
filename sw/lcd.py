from enum import Enum
from abc import ABC, abstractmethod
from time import sleep

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
    def __init__(self):
        self.team = Team.BLEU
        self.menu = None
        self.score = 0
        self.etatLidar = False
        self.tirette = False # True, tirée, False, encore en marche
        self.Mainmenu = MainMenu("MainMenu")
        self.stratMenu = MenuStrat("StratMenu")
        self.posMenu = PositionMenu.MAIN_MENU
        self.stratChoisie = False


    def initialisation(self):
        """Initialise le lcd"""
        self.menu = self.Mainmenu
        self.posMenu = PositionMenu.MAIN_MENU
        self.Mainmenu.initialisation()
        self.stratMenu.initialisation()


    def onReceivedTeam(self):
        """
        Lorsqu'on appuie sur le bouton la 1ere fois, on est bleu, si on rappuie on change de couleur
        """
        if self.team == Team.BLEU:
            self.team = Team.JAUNE
        elif self.team == Team.JAUNE:
            self.team = Team.BLEU
        else : 
            self.team = Team.BLEU

        self.send_msg(self.team)


    def send_msg(self,msg):
        """
        fonction qui envoie les données à la com
        """
        print(msg)


    def onReceivedTurn(self):
        """
        fonction qui envoie les données à la com
        """
        self.menu.turnPage()

    # TODO : On received Return

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
    


### TODO 
# Communication avec le bas niveau, qui va recevoir les messages, bridge_interface
# communication robot pour les infos importantes
# send score 
# d'autres à décider plus tard


if __name__ == "__main__": 
    ### test , simu
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