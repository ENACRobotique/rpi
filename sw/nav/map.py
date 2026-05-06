import matplotlib.pyplot as plt
import matplotlib.patches as patches
from math import sqrt
#import dijkstra
import random as rd 

RAYON_ROBOT = 200

# class data_linewidth_plot():
#     def __init__(self, x, y, **kwargs):
#         self.ax = kwargs.pop("ax", plt.gca())
#         self.fig = self.ax.get_figure()
#         self.lw_data = kwargs.pop("linewidth", 1)
#         self.lw = 1
#         self.fig.canvas.draw()

#         self.ppd = 72./self.fig.dpi
#         self.trans = self.ax.transData.transform
#         self.linehandle, = self.ax.plot([],[],**kwargs)
#         if "label" in kwargs: kwargs.pop("label")
#         self.line, = self.ax.plot(x, y, **kwargs)
#         self.line.set_color(self.linehandle.get_color())
#         self._resize()
#         self.cid = self.fig.canvas.mpl_connect('draw_event', self._resize)

#     def _resize(self, event=None):
#         lw =  ((self.trans((1, self.lw_data))-self.trans((0, 0)))*self.ppd)[1]
#         if lw != self.lw:
#             self.line.set_linewidth(lw)
#             self.lw = lw
#             self._redraw_later()

#     def _redraw_later(self):
#         self.timer = self.fig.canvas.new_timer(interval=10)
#         self.timer.single_shot = True
#         self.timer.add_callback(lambda : self.fig.canvas.draw_idle())
#         self.timer.start()

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
    """
    Lis le fichier fournit en paramètre, écrit de la manière suivante :
    NomPoint x y voisin1,voisin2,...
    renvoi le graph associé
    """
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


# def print_chemin(g,chemin):
#     """
#     Affiche le tracé du chemin donné en paramètre
#     """
#     for i in range(len(chemin)-1):
#         x1 = g.coords[chemin[i]][0]
#         y1 = g.coords[chemin[i]][1]
#         x2 = g.coords[chemin[i+1]][0]
#         y2 = g.coords[chemin[i+1]][1]
#         plt.plot([x1,x2],[y1,y2],color='r',linewidth=5)

