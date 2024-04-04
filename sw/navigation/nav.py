import map
import dijkstra

class Nav(object):  
    def __init__(self):
        """
        Outil permettant de gérer le déplacement du robot, le long d'un chemin, à partir d'un point
        d'entrée et d'un point de sortie
        """
        self.entree : str
        self.sortie : str
        self.graph : map.Graph
        self.file = "sw/navigation/graph.txt"
        self.path : list
        self.statut : bool # arrivé : True ; en chemin : False. # sert pour savoir si on peut envoyer la nouvelle consigne
        self.consigne : tuple # (x,y)
        self.current : tuple # point actuellement occupé


    def initialisation(self):
        """Initialise le graph et ses poids"""
        self.graph = map.read_graph(self.file) 
        self.graph.weight()
    

    def findPath(self):
        """Renvoi le chemin le plus cours entre l'entrée et la sortie"""
        self.chemin,distance_totale = dijkstra.dijkstra_classic(self.graph,self.entree, self.sortie) #a liste des points parcourus,nd distance parcourue
        # print(graph.coords[chemin[1]][0]) #coordonnes x du point 
        #pour obtenir les coords d'un point le la liste a : pt = g.coords["nom_du_point"]

    def send_cmd(self):
        """
        Envoi au robot les positions à atteindre fournies par "chemin"
        Tant que la position souhaitée n'est pas atteinte, on n'envoi pas la position suivante
        """
        for x in self.chemin:
            self.statut = False
            self.consigne = (x[0],x[1])
            while not self.statut :
                pass


#### Pour tester le système 
# nav = Nav()
# nav.initialisation()
# nav.entree = "secureB"
# nav.sortie = "secureJ"
# nav.findPath()
# nav.send_cmd()
### faire en sorte que le bas niveau renvoi si il est arrivé ou non
