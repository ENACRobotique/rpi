# import matplotlib.pyplot as plt
# import matplotlib.patches as patches
from math import sqrt
#import dijkstra
import random as rd 

RAYON_ROBOT = 200


class data_linewidth_plot():
    def __init__(self, x, y, **kwargs):
        self.ax = kwargs.pop("ax", plt.gca())
        self.fig = self.ax.get_figure()
        self.lw_data = kwargs.pop("linewidth", 1)
        self.lw = 1
        self.fig.canvas.draw()

        self.ppd = 72./self.fig.dpi
        self.trans = self.ax.transData.transform
        self.linehandle, = self.ax.plot([],[],**kwargs)
        if "label" in kwargs: kwargs.pop("label")
        self.line, = self.ax.plot(x, y, **kwargs)
        self.line.set_color(self.linehandle.get_color())
        self._resize()
        self.cid = self.fig.canvas.mpl_connect('draw_event', self._resize)

    def _resize(self, event=None):
        lw =  ((self.trans((1, self.lw_data))-self.trans((0, 0)))*self.ppd)[1]
        if lw != self.lw:
            self.line.set_linewidth(lw)
            self.lw = lw
            self._redraw_later()

    def _redraw_later(self):
        self.timer = self.fig.canvas.new_timer(interval=10)
        self.timer.single_shot = True
        self.timer.add_callback(lambda : self.fig.canvas.draw_idle())
        self.timer.start()

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


def print_map(graph, ax = None):
    """
    Affiche la map de la table avec les chemins du graph
    """
    g = graph 
    for point in g.adj :
        plt.annotate(point, [g.coords[point][0] + rd.randint(-2,2)*0, g.coords[point][1]+ rd.randint(-2,2)/25 ],fontsize=14)
        for voisin in g.adj[point]:
            x1 = g.coords[point][0]
            y1 = g.coords[point][1]
            x2 = g.coords[voisin][0]
            y2 = g.coords[voisin][1]
            plt.plot([x1,x2],[y1,y2],color='black')
            l = data_linewidth_plot([x1,x2],[y1,y2], ax=ax, solid_capstyle="round",linewidth = 2*RAYON_ROBOT, alpha = 0.2)

    #Affichage des bords de table
    table = patches.Rectangle((0,0),3000,2000,linewidth=1.5, edgecolor='black', facecolor='none')
    ax.add_patch(table)
    
    #Affichage des zones Jaunes
    j1 = patches.Rectangle((1000,   0),450,450,linewidth=1.5, facecolor='yellow', edgecolor='black')
    j2 = patches.Rectangle((2550, 650),450,450,linewidth=1.5, facecolor='yellow', edgecolor='black')
    j3 = patches.Rectangle(( 150,1550),450,450,linewidth=1.5, facecolor='yellow', edgecolor='black')
    ax.add_patch(j1)
    ax.add_patch(j2)
    ax.add_patch(j3)
    
    #Affichage des zones Bleu
    b1 = patches.Rectangle((1550,   0),450,450,linewidth=1.5, facecolor='lightblue', edgecolor='black')
    b2 = patches.Rectangle((   0, 650),450,450,linewidth=1.5, facecolor='lightblue', edgecolor='black')
    b3 = patches.Rectangle((2400,1550),450,450,linewidth=1.5, facecolor='lightblue', edgecolor='black')
    ax.add_patch(b1)
    ax.add_patch(b2)
    ax.add_patch(b3)

    #Affichage des stocks 
    s1  = patches.Rectangle((  25, 200),100,400,linewidth=1.5, facecolor='burlywood', edgecolor='black')
    s2  = patches.Rectangle((2875, 200),100,400,linewidth=1.5, facecolor='burlywood', edgecolor='black')
    s3  = patches.Rectangle((  25,1125),100,400,linewidth=1.5, facecolor='burlywood', edgecolor='black')
    s4  = patches.Rectangle((2875,1125),100,400,linewidth=1.5, facecolor='burlywood', edgecolor='black')
    s5  = patches.Rectangle(( 575, 200),400,100,linewidth=1.5, facecolor='burlywood', edgecolor='black')
    s6  = patches.Rectangle((2025, 200),400,100,linewidth=1.5, facecolor='burlywood', edgecolor='black')
    s7  = patches.Rectangle((1700, 900),400,100,linewidth=1.5, facecolor='burlywood', edgecolor='black')
    s8  = patches.Rectangle(( 900, 900),400,100,linewidth=1.5, facecolor='burlywood', edgecolor='black')
    s9  = patches.Rectangle(( 625,1675),400,100,linewidth=1.5, facecolor='burlywood', edgecolor='black')
    s10 = patches.Rectangle((1975,1675),400,100,linewidth=1.5, facecolor='burlywood', edgecolor='black')
    ax.add_patch(s1)
    ax.add_patch(s2)
    ax.add_patch(s3)
    ax.add_patch(s4)
    ax.add_patch(s5)
    ax.add_patch(s6)
    ax.add_patch(s7)
    ax.add_patch(s8)
    ax.add_patch(s9)
    ax.add_patch(s10)
    
    #Affichage des zones de construction par équipe
    c1 = patches.Rectangle((0,0),450,150,linewidth=1.5, facecolor='lightblue', edgecolor='black')
    c2 = patches.Rectangle((2000,0),450,150,linewidth=1.5, facecolor='lightblue', edgecolor='black')
    c3 = patches.Rectangle((555,0),450,150,linewidth=1.5, facecolor='yellow', edgecolor='black')
    c4 = patches.Rectangle((2550,0),450,150,linewidth=1.5, facecolor='yellow', edgecolor='black')
    ax.add_patch(c1)
    ax.add_patch(c2)
    ax.add_patch(c3)
    ax.add_patch(c4)

    #Affichage des rampes
    r1= patches.Rectangle((1050,1550),900,450, facecolor='purple', linewidth=1.5, edgecolor='black')
    r2= patches.Rectangle(( 650,1800),400,200, facecolor='purple', linewidth=1.5, edgecolor='black')
    r3= patches.Rectangle((1950,1800),400,200, facecolor='purple', linewidth=1.5, edgecolor='black')
    ax.add_patch(r1)
    ax.add_patch(r2)
    ax.add_patch(r3)


def print_chemin(g,chemin):
    """
    Affiche le tracé du chemin donné en paramètre
    """
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
    fig, ax = plt.subplots()
    print_map(graph,ax)

    #print_chemin(graph,chemin)

    #print(chemin)
    #print(distance_totale)

    #print(graph.coords[chemin[1]][0]) #coordonnes x du point 
    #pour obtenir les coords d'un point le la liste a : pt = g.coords["nom_du_point"]
    # plt.grid(True, color='black', linestyle='--', linewidth=0.5)
    plt.title("Carte Très Basique",fontsize=16)
    plt.xticks(size=16)
    plt.yticks(size=16)
    plt.axis('equal')
    plt.show()