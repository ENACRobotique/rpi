from enum import Enum
import time 

class State(Enum):
    INITIALISATION = 0
    NAVIGATION = 1
    ARRET = 2
    RAMASSE_PLANTE = 3
    DEPOSE = 4


class Strat:
    """
    Classe permettant de créer une machine à état dynamique, capable de revenir à l'état précédent etc
    """
    #attributs
    current_state : str 
    previous_state : str


    def __init__(self):
        self.current_state = State.INITIALISATION


    def case_initialisation(self):
        """
        initialisation : à preciser plus tard
        """
        #mise à jour des états actuels et précédents
        self.previous_state = self.current_state
        self.current_state = State.INITIALISATION
        #initialisation du robot
        t0 = time.time()
        t = time.time()
        while (t - t0) < 3:
            t = time.time()
        print("j'ai suit inissialisey")

        #Passage à l'état suivant 
        self.case_nav("plantes1")


    def case_nav(self,point:str):
        """
        Voyage de la position actuelle au point passé en paramètre
        """
        #mise à jour des états actuels et précédents
        self.previous_state = self.current_state
        self.current_state = State.NAVIGATION

        #GO TO
        x,y = 0.5, 0.7  #points trouvé dans la liste des points du graphe 
        #going to
        t0 = time.time()
        t = time.time()
        while (t - t0) < 3:
            t = time.time()
        print("j'ai suit arrivé")

        #Prochaine action suivant l'état précédent

        if self.previous_state == State.INITIALISATION:
            self.case_ramasse_plante()
        elif  self.previous_state == State.RAMASSE_PLANTE:
            self.case_depose()
        elif  self.previous_state == State.DEPOSE:
            self.case_arret()
        

    def case_ramasse_plante(self):
        """
        Envoie les consignes bas niveau pour le ramassage des plantes
        """
        #mise à jour des états actuels et précédents
        self.previous_state = self.current_state
        self.current_state = State.DEPOSE

        #ramasse ses plantes
        #going to
        t0 = time.time()
        t = time.time()
        while (t - t0) < 3:
            t = time.time()
        print("plantes ramassées")

        #Prochaine action
        self.case_nav("pt_de_depose")


    def case_depose(self):
        """
        Envoie les consignes bas niveau de ramassage des plantes
        """
        #mise à jour des états actuels et précédents
        self.previous_state = self.current_state
        self.current_state = State.RAMASSE_PLANTE

        #ramasse ses plantes
        #going to
        t0 = time.time()
        t = time.time()
        while (t - t0) < 1:
            t = time.time()
        print("plantes déposée")

        #Prochaine action
        self.case_nav("pt_de_depose")


    def case_arret(self):
        """
        Arret total de systeme dans la zone de recharge des batteries
        """
        #mise à jour des états actuels et précédents
        self.previous_state = self.current_state
        self.current_state = State.ARRET

        #ramasse ses plantes
        #going to
        t0 = time.time()
        t = time.time()
        while (t - t0) < 0.5:
            t = time.time()
        print("arrêté")





strat = Strat()
strat.case_initialisation()



