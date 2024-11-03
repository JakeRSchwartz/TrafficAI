import heapq
import math

class RouteAlgo:
    def __init__(self, graph, G):
        self.graph = graph
        self.G = G

    # Euclidean distance as the heuristic function
    def heuristic(self, node1, node2):
        x1, y1 = self.G.nodes[node1]['x'], self.G.nodes[node1]['y']
        x2, y2 = self.G.nodes[node2]['x'], self.G.nodes[node2]['y']
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def a_star(self, start, goal):
        # Priority queue for A*
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        # Track the cost of reaching each node
        g_cost = {node: float('inf') for node in self.graph}
        g_cost[start] = 0
        
        # Track the total estimated cost to reach the goal
        f_cost = {node: float('inf') for node in self.graph}
        f_cost[start] = self.heuristic(start, goal)
        
        # Store the path
        came_from = {}
        
        while open_set:
            # Get the node with the lowest f_cost
            _, current = heapq.heappop(open_set)
            
            if current == goal:
                # Reconstruct the path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]  # Return reversed path
            
            for neighbor, distance in self.graph[current].items():
                tentative_g_cost = g_cost[current] + distance
                
                if tentative_g_cost < g_cost[neighbor]:
                    # Update path
                    came_from[neighbor] = current
                    g_cost[neighbor] = tentative_g_cost
                    f_cost[neighbor] = tentative_g_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_cost[neighbor], neighbor))
                    
        return None  # If there's no path


    def dijkstra(self, start, goal):
        # Priority queue for Dijkstra's
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        # Track the cost of reaching each node
        g_cost = {node: float('inf') for node in self.graph}
        g_cost[start] = 0
        
        # Store the path
        came_from = {}
        
        while open_set:
            current_cost, current = heapq.heappop(open_set)
            
            if current == goal:
                # Reconstruct the path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]  # Return reversed path
            
            for neighbor, distance in self.graph[current].items():
                tentative_g_cost = current_cost + distance
                
                if tentative_g_cost < g_cost[neighbor]:
                    # Update path
                    came_from[neighbor] = current
                    g_cost[neighbor] = tentative_g_cost
                    heapq.heappush(open_set, (tentative_g_cost, neighbor))
                    
        return None  # If there's no path

    

