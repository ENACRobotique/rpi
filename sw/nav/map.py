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
        self.adj_removed = {}
        self.node_removed = {}

    def __repr__(self):
        """Representation en chaine de caracteres d'un graphe"""
        return '<Graph: {0.adj}>'.format(self)

    def add_node(self, u,x,y):
        """Ajoute un noeud u au graphe"""
        self.adj[u] = []
        self.coords[u] = [x,y]

    def remove_node(self, u):
        self.node_removed.update({u,self.coords[u], self.adj[u]})
        for v in self.adj[u]:
            self.remove_edge(u,v)


    def add_edge(self, u, v):
        """Ajoute l'arete (u, v) au graphe si elle n'est pas présente"""
        if not (v in self.adj.get(u, [])):
            self.adj.setdefault(u, []).append(v)
        if not (u in self.adj.get(v, [])):
            self.adj.setdefault(v, []).append(u)

    def re_add_edge(self, u, v):
        """Ajoute l'arete (u, v) au graphe uniquement si elle était dans les arretes supprimées"""
        if v in self.adj_removed.get(u, []):
            self.adj.setdefault(u, []).append(v)
            self.adj_removed[u].remove(v)
        if u in self.adj_removed.get(v, []):
            self.adj.setdefault(v, []).append(u)
            self.adj_removed[v].remove(u)


    def remove_edge(self, u, v):
        """Retire l'arete (u, v) au graphe si elle est présente"""
        if v in self.adj.get(u, []):
            self.adj[u].remove(v)
            self.adj_removed.setdefault(u, []).append(v)
        if u in self.adj.get(v, []):
            self.adj[v].remove(u)
            self.adj_removed.setdefault(v, []).append(u)


    def arrete_proche_point(self, u, v, x0, y0, seuil, proche = True):
        """ renvoie si l'arrete u,v est trop proche ou trop loin de x0,y0 
            + proche = 
                + True -> trop proche (< seuil)
                + False -> loin (>= seuil)"""

        xu, yu = self.coords[u]
        xv, yv = self.coords[v]

        uvx = xv - xu
        uvy = yv - yu

        acx = x0 - xu
        acy = y0 - yu

        uv2 = uvx * uvx + uvy * uvy

        # segment réduit à un point
        if uv2 == 0:
            return False

        t = (acx * uvx + acy * uvy) / uv2

        # entre 0 et 1
        t = max(0, min(1, t))

        px = xu + t * uvx
        py = yu + t * uvy

        if proche:
            return (x0 - px)**2 + (y0 - py)**2 < seuil**2
        else :
            return (x0 - px)**2 + (y0 - py)**2 >= seuil**2
                

    def calc_arretes_proche_point(self, arrete_list, x0, y0, seuil, proche = True):
        """ Renvoie les arretes proche ou loin du point (x0, y0)
            + proche = 
                + True -> trop proche (< seuil)
                + False -> loin (>= seuil)"""        
        res = []
        for u in arrete_list:
            for v in arrete_list[u]:
                #evite de traiter 2 fois la meme arrete
                if u > v:
                    continue

                if self.arrete_proche_point(u,v,x0,y0, seuil, proche):
                    res.extend([(u, v)])
        return res

    def node_proche_point(self, u, x0, y0, seuil, proche = True):
        """ Renvoie si le noeud est proche ou loin du point (x0, y0)
            + proche = 
                + True -> trop proche (< seuil)
                + False -> loin (>= seuil)"""        
        x, y = self.coords[u]
        if proche:
            return (x - x0)**2 + (y - y0)**2 < seuil
        else :
            return (x - x0)**2 + (y - y0)**2 >= seuil

    def calc_node_proche_point(self, node_list, x0, y0, seuil, proche = True):
        for node in node_list:
            if self.node_proche_point(node, x0, y0, seuil, proche):
                pass


              
    def update_edge_pos(self, x0, y0, seuil):
            arrete_adj_trop_proche_point = self.calc_arretes_proche_point(self.adj, x0, y0, seuil, True)
            arrete_removed_trop_loin_point = self.calc_arretes_proche_point(self.adj_removed, x0, y0, seuil, False)
            for u,v in arrete_adj_trop_proche_point:
                self.remove_edge(u, v)
            for u,v in arrete_removed_trop_loin_point:
                self.re_add_edge(u,v)
            



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

