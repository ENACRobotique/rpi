import heapq

class PriorityQueue(object):
    def __init__(self, initial_list : list):
        """
        Crée une file à priorité à partir d'une liste initiale.
        Chaque élément de la liste initiale doit être un tuple (priorité, élément).
        """
        self.queue = [[priority, index, element] for (index, (priority, element)) in enumerate(initial_list)]
        heapq.heapify(self.queue)

    def __repr__(self):
        """
        Représentation en chaîne de la file à priorité.
        """
        return '<PriorityQueue: {0.queue}>'.format(self)

    def decrease_priority(self, element, new_priority):
        """
        Met à jour la priorité d'un élément dans la file à priorité en remplaçant son ancienne priorité par une nouvelle.
        """
        for index, (_, _, e) in enumerate(self.queue):
            if e == element:
                self.queue[index][0] = new_priority
                heapq._siftdown(self.queue, 0, index) #type: ignore
        return self.queue
        raise KeyError("decrease_priority")

    def pop(self):
        """
        Extrait l'élément avec la plus petite priorité de la file à priorité et le renvoie.
        """
        priority, _, element = heapq.heappop(self.queue)
        return (priority, element)

    def is_empty(self):
        """
        Vérifie si la file à priorité est vide.
        """
        return len(self.queue) == 0


INFINITY = 10000

def initialize_distances(graph, start_node):
    """
    Initialise les distances à partir du noeud de départ dans le graphe.
    """
    nodes = graph.nodes()
    distances = {}
    for node in nodes:
        distances[node] = INFINITY
    distances[start_node] = 0
    return distances


def find_min(priority_queue : PriorityQueue):
    """
    Trouve et renvoie l'élément avec la plus petite priorité dans la file à priorité.
    """
    priority, element = priority_queue.pop()
    return element


def update_distances(source, destination, predecessors : dict, distances : dict , graph):
    """
    Met à jour la distance entre deux noeuds dans le graphe.
    """
    weight = graph.weights[(source, destination)]
    if distances[destination] > distances[source] + weight :
        distances[destination] = distances[source] + weight
        predecessors[destination] = source
    return distances, predecessors


def dijkstra_classic(graph, start_node, end_node):
    """
    Applique l'algorithme de Dijkstra pour trouver le chemin le plus court entre le noeud de départ et le noeud de fin dans le graphe.
    Returns:
    tuple - Une paire contenant le chemin le plus court (une liste de noeuds successifs) et la longueur de ce chemin.
    """
    # Initialisation des distances à partir du noeud de départ
    distances = initialize_distances(graph, start_node)

    # Récupération de tous les noeuds dans le graphe
    nodes = graph.nodes()

    # Création de la liste de priorité initiale pour la file à priorité
    initial_priority_queue = [(distances[node], node) for node in nodes]

    # Création de la file à priorité
    priority_queue = PriorityQueue(initial_priority_queue)
    
    # Dictionnaire pour stocker les prédécesseurs de chaque noeud
    predecessors = {}

    # Liste pour stocker les noeuds visités
    visited_nodes = []

    # Boucle principale de l'algorithme de Dijkstra
    while not priority_queue.is_empty():
        # Sélection du noeud avec la plus petite distance actuelle dans la file à priorité
        current_node = find_min(priority_queue)
        visited_nodes.append(current_node)

        # Si on atteint le noeud de fin, on arrête la recherche
        if current_node == end_node:
            break

        # Parcours des voisins du noeud courant
        for neighbor_node in graph.neighbours(current_node):
            # Mise à jour des distances et des prédécesseurs
            update_distances(current_node, neighbor_node, predecessors, distances, graph)
            priority_queue.decrease_priority(neighbor_node, distances[neighbor_node])

    # Reconstruction du chemin le plus court à partir des prédécesseurs
    shortest_path = []
    current = end_node
    while current != start_node: 
        shortest_path.append(current)
        current = predecessors[current]
    shortest_path.append(start_node)

    # Retour du chemin le plus court et de sa longueur
    return list(reversed(shortest_path)), distances[end_node]