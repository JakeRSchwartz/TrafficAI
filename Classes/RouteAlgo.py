from collections import deque
import heapq
import math
from collections import deque

class RouteAlgo:
    def __init__(self, graph, G):
        self.graph = graph
        self.G = G
    
    def a_star(self, start, goal, metric):
        def heuristic(node1, node2):
            # Calculate heuristic based on metric
            x1, y1 = self.G.nodes[node1]['x'], self.G.nodes[node1]['y']
            x2, y2 = self.G.nodes[node2]['x'], self.G.nodes[node2]['y']
            distance = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

            if metric == 'travel_time_seconds':
                # Use the fastest speed in the graph for the heuristic
                max_speed = max(edge_data['maxspeed'] for node in self.graph for edge_data in self.graph[node].values())
                max_speed_mps = max_speed * 0.44704  # Convert max_speed to meters/second
                return distance / max_speed_mps if max_speed_mps > 0 else float('inf')
            return distance  # Default heuristic for distance

        open_set = []
        heapq.heappush(open_set, (0, start))  # (f_cost, node)

        came_from = {}
        g_cost = {node: float('inf') for node in self.graph}
        g_cost[start] = 0

        f_cost = {node: float('inf') for node in self.graph}
        f_cost[start] = heuristic(start, goal)

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1], g_cost[goal]

            for neighbor, edge_data in self.graph[current].items():
                cost = edge_data[metric] 
                tentative_g_cost = g_cost[current] + cost

                if tentative_g_cost < g_cost[neighbor]:
                    came_from[neighbor] = current
                    g_cost[neighbor] = tentative_g_cost
                    f_cost[neighbor] = tentative_g_cost + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_cost[neighbor], neighbor))

        return None, float('inf')  # Return None if no path is found

    def dijkstra(self, start, goal, metric):
        priority_queue = []
        heapq.heappush(priority_queue, (0, start))  # (total_cost, node)

        came_from = {}
        g_cost = {node: float('inf') for node in self.graph}
        g_cost[start] = 0

        while priority_queue:
            current_cost, current = heapq.heappop(priority_queue)

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1], g_cost[goal]

            for neighbor, edge_data in self.graph[current].items():
                edge_cost = edge_data[metric]
                tentative_cost = g_cost[current] + edge_cost

                if tentative_cost < g_cost[neighbor]:
                    came_from[neighbor] = current
                    g_cost[neighbor] = tentative_cost
                    heapq.heappush(priority_queue, (tentative_cost, neighbor))

        return None, float('inf')  # Return None if no path is found

    def bfs(self, start, goal, metric):
        queue = deque([(start, [start], 0)])  # (current_node, path, total_cost)
        visited = set()

        while queue:
            current, path, cost = queue.popleft()

            if current in visited:
                continue
            visited.add(current)

            if current == goal:
                return path, cost  # Return path and total cost

            for neighbor, edge_data in self.graph[current].items():
                if neighbor not in visited:
                    edge_cost = edge_data[metric]
                    queue.append((neighbor, path + [neighbor], cost + edge_cost))

        return None, float('inf')  # Return None if no path is found

    def dfs(self, current, goal, metric, visited=None, path=None, cost=0):
        if visited is None:
            visited = set()
        if path is None:
            path = []

        # Mark the current node as visited and add to the path
        visited.add(current)
        path.append(current)

        # If the goal is reached, return the path and the cost
        if current == goal:
            return path, cost

        # Explore neighbors
        for neighbor, edge_data in self.graph[current].items():
            if neighbor not in visited:
                edge_cost = edge_data[metric]
                result_path, result_cost = self.dfs(
                    neighbor, goal, metric, visited, path, cost + edge_cost
                )
                if result_path:
                    return result_path, result_cost

        # Backtrack if no path is found
        path.pop()
        return None, float('inf')

    

        

        

        

        

