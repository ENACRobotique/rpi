import map
import dijkstra
import matplotlib.pyplot as plt
from math import sqrt
class Nav(object):  
    def __init__(self):
        """
        Outil permettant de gérer le déplacement du robot, le long d'un chemin, à partir d'un point
        d'entrée et d'un point de sortie
        """
        self.entree : str
        self.sortie : str
        self.graph : map.Graph
        self.file = "graph.txt"
        self.path : list
        self.statut : bool # arrivé : True ; en chemin : False. # sert pour savoir si on peut envoyer la nouvelle consigne
        self.consigne : tuple # (x,y)
        self.current : tuple # point actuellement occupé


    def initialisation(self):
        """Initialise le graph et ses poids"""
        self.graph = map.read_graph(self.file) 
        self.graph.weight()
        ## On augmente les poids de la "croix" au milieu pour ne pas passer dans les plantes
        self.graph.weights[('planteNW', 'mid')] += 10000
        self.graph.weights[('mid', 'planteNW')] += 10000
        self.graph.weights[('planteNE', 'mid')] += 10000
        self.graph.weights[('mid', 'planteNE')] += 10000
        self.graph.weights[('navSW', 'mid')] += 10000
        self.graph.weights[('mid', 'navSW')] += 10000
        self.graph.weights[('navSE', 'mid')] += 10000
        self.graph.weights[('mid', 'navSE')] += 10000
    
    def getCoords(self,waypoint):
        """Unité en milimètres !!!!"""
        x,y = self.graph.coords[waypoint]
        #print(f"Waypoint {waypoint} is : {x}\t{y} \n")
        return x,y

    def findPath(self):
        """Renvoi le chemin le plus cours entre l'entrée et la sortie"""
        self.chemin,distance_totale = dijkstra.dijkstra_classic(self.graph,self.entree, self.sortie) #a liste des points parcourus,nd distance parcourue
        # print(graph.coords[chemin[1]][0]) #coordonnes x du point 
        #pour obtenir les coords d'un point le la liste a : pt = g.coords["nom_du_point"]
    
    def resetPath(self):
        self.chemin = []
    
    def closestWaypoint(self,x,y):
        """ Renvoie le waypoint le plus proche en terme de norme """
        m=[]
        for w in self.graph.coords:
            m.append((sqrt((self.graph.coords[w][0]-x)**2 + (self.graph.coords[w][1]-y)**2 ),w))
            #print(m[-1])
        print("closest is ", min(m)[1])
        return min(m)[1]


if __name__ == "__main__" : 

    #### Pour tester le système 
    nav = Nav()
    nav.initialisation()
    nav.entree = "secureB"
    nav.sortie = "secureJ"
    nav.findPath()
    #print(nav.closestWaypoint(100,200))
    plt.plot()

    
    ### faire en sorte que le bas niveau renvoi si il est arrivé ou non
