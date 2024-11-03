import osmnx as ox
from classes.RouteAlgo import RouteAlgo 
from classes.Helpers import Helpers


def main():
    #Get Graph
    #G = ox.graph_from_place("Manhattan, New York, USA", network_type="drive")

    # Load the graph from a GraphML file
    G = ox.load_graphml("./data/manhattan_drive.graphml")
    
    # Euclidean distance as the heuristic function
    graph = Helpers.CreateAdjList(G)

    # Create an instance of RouteAlgo with the graph
    route_algo = RouteAlgo(graph, G)

    # Define your locations by name
    # We could get user input to make it seem more interative.....
    start_location = "Empire State Building, New York, NY"
    end_location = "Times Square, New York, NY"

    # Get coordinates for each location
    start_coords = Helpers.GetCoords(start_location)
    end_coords = Helpers.GetCoords(end_location)
    
    # Define start and goal nodes
    start = ox.distance.nearest_nodes(G, X=start_coords[1], Y=start_coords[0])
    goal = ox.distance.nearest_nodes(G, X=end_coords[1], Y=end_coords[0])
    
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


    
    

