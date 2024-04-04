import heapq

class PrioQueue(object):
    def __init__(self, list : list):
        """
        Créer une file à priorité à partir d'une liste list :
        la liste list doit contenir les couples (priorité, element)
        """
        self.queue = [[prio, i, elt] for (i, (prio, elt)) in enumerate(list)]
        heapq.heapify(self.queue)


    def __repr__(self):
        """
        Représentation chaine de la file a priorite
        """
        return '<PrioQueue: {0.queue}>'.format(self)


    def decrease_prio(self, elt, new_prio):
        """
        Met à jour la priorite de 'elt' dans la file
        en la remplaçant par 'new_prio'
        """
        for i, (_, _, e) in enumerate(self.queue):
            if e == elt:
                self.queue[i][0] = new_prio
                heapq._siftdown(self.queue, 0, i) #type: ignore
        return self.queue
        raise KeyError("decrease_prio")


    def pop(self):
        """
        Extrait de la file à priorité l'élement de priorité minimale
        Renvoie le couple (priorité, élement) correspondant
        """
        prio, _, elt = heapq.heappop(self.queue)
        return (prio, elt)


    def is_empty(self):
        """
        Teste si la file à priorité est vide
        """
        return len(self.queue) == 0


INF = 10000

def initialisation(G,start):
    L = G.nodes()
    d = {}
    for u in L:
        d[u]= INF
    d[start]=0
    
    return d


def find_min(Q : PrioQueue):
    a=Q
    b = a.pop()
    return b[1]


def maj_dist(s1, s2, predecesseur : dict, d :dict , G):
    a = G.weights[(s1,s2)]
    if d[s2] > d[s1] + a[0] :
        d[s2] = d[s1] + a[0]
        predecesseur[s2] = s1

    return d,predecesseur


def dijkstra_classic(G,start,end):  #G un graphe
    """
    return le chemin le plus court (liste de noeuds successif) et la longueur de ce chemin
    """
    d = initialisation(G,start)
    L = G.nodes()
    Lq = [(d[u],u) for u in L]
    Q = PrioQueue(Lq)
    
    predecesseur ={}
    visite = []
    while not Q.is_empty():
        u = find_min(Q)
        visite.append(u)
        if u == end :
            break
        for v in G.neighbours(u):
            maj_dist(u,v,predecesseur,d,G)
            Q.decrease_prio(v,d[v])
    chemin = []
    s = end
    while s!=start : 
        chemin +=[s]
        s = predecesseur[s]
    chemin += [start]

    return list(reversed(chemin)), d[end]