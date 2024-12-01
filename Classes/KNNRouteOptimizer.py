import heapq
from sklearn.neighbors import NearestNeighbors

class KNNRouteOptimizer:
    def __init__(self, graph, G, k=5):
        """
        Initialize the KNN route optimizer with a graph and spatial representation.
        :param graph: The adjacency list containing edge attributes like 'length' or 'travel_time_seconds'.
        :param G: The original graph (osmnx format) for node coordinates.
        :param k: Number of neighbors to consider for KNN.
        """
        self.graph = graph
        self.G = G
        self.k = k
        self.node_coords = self._get_node_coordinates()
        self.knn_model = NearestNeighbors(n_neighbors=k, algorithm='ball_tree').fit(self.node_coords)

    def _get_node_coordinates(self):
        """
        Extract node coordinates from the graph for KNN fitting.
        :return: List of [x, y] coordinates for each node.
        """
        return [[self.G.nodes[node]['x'], self.G.nodes[node]['y']] for node in self.G.nodes]

    def knn_route(self, start, goal, metric='length'):
        """
        Find a route using KNN to explore nearest neighbors at each step.
        :param start: Starting node ID.
        :param goal: Goal node ID.
        :param metric: Edge attribute to minimize (e.g., 'length' or 'travel_time_seconds').
        :return: Path as a list of nodes and the total cost.
        """
        current_node = start
        visited = set()
        path = [current_node]
        total_cost = 0

        while current_node != goal:
            # Mark the current node as visited
            visited.add(current_node)

            # Get the coordinates of the current node
            current_coords = [[self.G.nodes[current_node]['x'], self.G.nodes[current_node]['y']]]

            # Find the K nearest neighbors
            distances, indices = self.knn_model.kneighbors(current_coords)
            neighbors = [list(self.G.nodes)[i] for i in indices[0]]  # Map indices to node IDs

            # Select the best neighbor not yet visited
            best_neighbor = None
            min_cost = float('inf')
            for neighbor in neighbors:
                if neighbor in visited:
                    continue  # Skip visited nodes
                if neighbor in self.graph[current_node]:
                    cost = self.graph[current_node][neighbor].get(metric, float('inf'))
                    if cost < min_cost:
                        min_cost = cost
                        best_neighbor = neighbor

            if best_neighbor is None:
                print("No valid path found!")
                return None, float('inf')

            # Update path and cost
            path.append(best_neighbor)
            total_cost += min_cost
            current_node = best_neighbor

        return path, total_cost
