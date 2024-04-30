import map
import dijkstra
import matplotlib.pyplot as plt
from math import sqrt,pi
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

    def findPath(self, theta_start, theta_dest):
        """Renvoi le chemin le plus cours entre l'entrée et la sortie -> une liste de tuple des positions (x,y,theta) a faire"""
        dtheta =theta_dest-theta_start
        if dtheta < -pi :
            theta_start -= 2*pi
        elif dtheta > pi :
            theta_start += 2*pi

        self.chemin,self.distance_totale = dijkstra.dijkstra_classic(self.graph,self.entree, self.sortie) #a liste des points parcourus,nd distance parcourue
        dist = 0 
        x,y = self.getCoords(self.chemin[0])
        pos=[(x,y,theta_start)]
        
        for i in range(1,len(self.chemin)):
            x1,y1 = self.getCoords(self.chemin[i-1])
            x2,y2 = self.getCoords(self.chemin[i])
            dist += sqrt((x2-x1)**2+(y2-y1)**2)
            theta = theta_start + (dtheta)*dist/self.distance_totale
            pos.append((x2,y2,theta))
        
        #print("Positions",pos)
        return pos
        

        #print("distance", self.distance_totale)
        # print(graph.coords[chemin[1]][0]) #coordonnes x du point 
        #pour obtenir les coords d'un point le la liste a : pt = g.coords["nom_du_point"]
    
    def resetPath(self):
        self.chemin = []
    
    def closestWaypoint(self,x,y):
        """ Renvoie le waypoint le plus proche en terme de norme """
        def dist_from_xy(w):
            w_x, w_y = self.graph.coords[w]
            return sqrt((w_x-x)**2 + (w_y-y)**2 )
        
        return min(self.graph.coords, key=dist_from_xy)

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
