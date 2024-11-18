import heapq
import math

class RouteAlgo:
    def __init__(self, graph, G):
        self.graph = graph
        self.G = G

    # Euclidean distance as the heuristic function
    def heuristic_distance(self, node1, node2):
        x1, y1 = self.G.nodes[node1]['x'], self.G.nodes[node1]['y']
        x2, y2 = self.G.nodes[node2]['x'], self.G.nodes[node2]['y']
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    def heuristic_time(self, node1, node2, average_speed=5):
        # Use distance heuristic and an assumed average speed to estimate travel time
        dist = self.heuristic_distance(node1, node2)
        return dist / (average_speed * 0.44704)  # Convert mph to m/s if needed
    
        
    def a_star_distance(self, start, goal):
        # Priority queue for A*
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        # Track the cost of reaching each node
        g_cost = {node: float('inf') for node in self.graph}
        g_cost[start] = 0
        
        # Track the total estimated cost to reach the goal
        f_cost = {node: float('inf') for node in self.graph}
        f_cost[start] = self.heuristic_distance(start, goal)
        
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
                    f_cost[neighbor] = tentative_g_cost + self.heuristic_distance(neighbor, goal)
                    heapq.heappush(open_set, (f_cost[neighbor], neighbor))
                    
        return None  # If there's no path
    
    def a_star_time(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))

        g_cost = {node: float('inf') for node in self.graph}
        g_cost[start] = 0

        f_cost = {node: float('inf') for node in self.graph}
        f_cost[start] = self.heuristic_time(start, goal)

        came_from = {}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for neighbor, data in self.graph[current].items():
                tentative_g_cost = g_cost[current] + data['predicted_time']

                if tentative_g_cost < g_cost[neighbor]:
                    came_from[neighbor] = current
                    g_cost[neighbor] = tentative_g_cost
                    f_cost[neighbor] = tentative_g_cost + self.heuristic_time(neighbor, goal)
                    heapq.heappush(open_set, (f_cost[neighbor], neighbor))

        return None


    def dijkstra_distance(self, start, goal):
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

    def dfs_distance(self, start, goal, path=[]):
        path = path + [start]
        if start == goal:
            return path
        for node, data in self.graph[start].items():
            if node not in path:
                new_path = self.dfs_distance(node, goal, path)
                if new_path:
                    return new_path
        return None
    
    def bfs_distance(self, start, goal):
        queue = [(start, [start])]
        
        while queue:
            current, path = queue.pop(0)
            if current == goal:
                return path
            
            for neighbor, data in self.graph[current].items():
                if neighbor not in path:
                    queue.append((neighbor, path + [neighbor]))
                    
        return None
    

    

    

