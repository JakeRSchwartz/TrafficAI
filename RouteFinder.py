import osmnx as ox
from Classes.RouteAlgo import RouteAlgo 
from Classes.Helpers import Helpers


def main():
    # Load the graph from a GraphML file
    G = ox.load_graphml("./data/manhattan_drive.graphml")
    
    # Euclidean distance as the heuristic function
    graph = Helpers.CreateAdjList(G)

    # Create an instance of RouteAlgo with the graph
    route_algo = RouteAlgo(graph, G)
    
    # Define start and goal nodes
    start = list(G.nodes())[0]
    goal = list(G.nodes())[-1]
    
    # Run A* algorithm
    astar_path = route_algo.a_star(start, goal)
    print("A* Path:", astar_path)
    if astar_path:
        print("A* Path length:", sum(graph[astar_path[i]][astar_path[i + 1]] for i in range(len(astar_path) - 1)))
        ox.plot_graph_route(G, astar_path, route_linewidth=2, node_size=0)

    # Run Dijkstra algorithm
    dijkstra_path = route_algo.dijkstra(start, goal)
    print("Dijkstra Path:", dijkstra_path)
    if dijkstra_path:
        print("Dijkstra Path length:", sum(graph[dijkstra_path[i]][dijkstra_path[i + 1]] for i in range(len(dijkstra_path) - 1)))
        ox.plot_graph_route(G, dijkstra_path, route_linewidth=2, node_size=0)

if __name__ == "__main__":
    main()


    
    

