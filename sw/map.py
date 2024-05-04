import matplotlib.pyplot as plt
from math import sqrt
import dijkstra
import random as rd 

class Graph(object):  
    def __init__(self):
        """Cree un graphe vide"""
        self.adj = {}
        self.coords = {}
        self.weights = {}

    def __repr__(self):
        """Representation en chaine de caracteres d'un graphe"""
        return '<Graph: {0.adj}>'.format(self)

    def add_node(self, u,x,y):
        """Ajoute un noeud u au graphe"""
        self.adj[u] = []
        self.coords[u] = [x,y]

    def add_edge(self, u, v):
        """Ajoute l'arete (u, v) au graphe"""
        self.adj.setdefault(u, []).append(v)
        self.adj.setdefault(v, []).append(u)

    def size(self):
        """Renvoie le nombre de noeuds du graphe"""
        return len(self.adj)

    def nodes(self):
        """Renvoie la liste des noeuds du graphe"""
        return self.adj.keys()

    def neighbours(self, u):
        """Renvoie la liste des voisins du noeud u dans le graphe"""
        return self.adj[u]

    def weight(self):
        self.weights = {}  # Réinitialisez weights pour éviter la duplication
        
        for point in self.adj:
            for voisin in self.adj[point]:
                x1, y1 = self.coords[point]
                x2, y2 = self.coords[voisin]
                w = sqrt((x1 - x2)**2 + (y1 - y2)**2)
                self.weights[(point, voisin)] = w
                self.weights[(voisin, point)] = w
        


def read_graph(file):
    g = Graph()
    neighbours = []
    points = []
    with open(file) as f:
        for line in f:
            l = line.split()
            g.add_node(l[0],float(l[1]),float(l[2])) #nom x y 
            points.append(l[0])
            nom_spliter = l[3].split(',')
            neighbours.append(nom_spliter)
    for count, x in enumerate(neighbours):
        for y in x:
            g.add_edge(points[count],y)
    return g


def print_map(graph):
    g = graph 
    for point in g.adj :
        plt.annotate(point, [g.coords[point][0] + rd.randint(-2,2)*0, g.coords[point][1]+ rd.randint(-2,2)/25 ])
        for voisin in g.adj[point]:
            x1 = g.coords[point][0]
            y1 = g.coords[point][1]
            x2 = g.coords[voisin][0]
            y2 = g.coords[voisin][1]
            plt.plot([x1,x2],[y1,y2],color='black')
    #affichage des jardinières
    plt.plot([0,0],[450,450+325],color='y')
    plt.plot([0,0],[2000-450-325,2000-450],color='b')
    plt.plot([600,925],[2000,2000],color='b')

    plt.plot([3000,3000],[450,450+325],color='b')
    plt.plot([3000,3000],[2000-450-325,2000-450],color='y')
    plt.plot([3000-600,3000-925],[2000,2000],color='y')

    
            


def print_chemin(g,chemin):
    for i in range(len(chemin)-1):
        x1 = g.coords[chemin[i]][0]
        y1 = g.coords[chemin[i]][1]
        x2 = g.coords[chemin[i+1]][0]
        y2 = g.coords[chemin[i+1]][1]
        plt.plot([x1,x2],[y1,y2],color='r',linewidth=5)
        
        

if __name__ == "__main__" : 
    ### test 
    file = "sw/graph.txt"
    graph = read_graph(file) #map de la table
    graph.weight()

    #chemin,distance_totale = dijkstra.dijkstra_classic(graph,"secureB", "potSE") #a liste des points parcourus,nd distance parcourue
    
    print(graph.weights)
    plt.figure()
    print_map(graph)

    #print_chemin(graph,chemin)

    #print(chemin)
    #print(distance_totale)

    #print(graph.coords[chemin[1]][0]) #coordonnes x du point 
    #pour obtenir les coords d'un point le la liste a : pt = g.coords["nom_du_point"]
    plt.grid()
    plt.axis('equal')
    plt.show()