import osmnx as ox
from classes.RouteAlgo import RouteAlgo 
from classes.Helpers import Helpers


def PrintPath(start, goal, graph, G, route_algo):
    # Run A* algorithm
    astar_path = route_algo.a_star_distance(start, goal)
    if astar_path:
        astar_length_miles = sum(graph[astar_path[i]][astar_path[i + 1]] for i in range(len(astar_path) - 1)) / 1609.34
        print(f"A* Path length (Miles): {astar_length_miles:.2f}")
        ox.plot_graph_route(G, astar_path, route_linewidth=2, node_size=0)

    # Run Dijkstra algorithm
    dijkstra_path = route_algo.dijkstra_distance(start, goal)
    if dijkstra_path:
        dijkstra_length_miles = sum(graph[dijkstra_path[i]][dijkstra_path[i + 1]] for i in range(len(dijkstra_path) - 1)) / 1609.34
        print(f"Dijkstra Path length (Miles): {dijkstra_length_miles:.2f}")
        ox.plot_graph_route(G, dijkstra_path, route_linewidth=2, node_size=0)
    
    dfs_path = route_algo.dfs_distance(start, goal)
    if dfs_path:
        dfs_length_miles = sum(graph[dfs_path[i]][dfs_path[i + 1]] for i in range(len(dfs_path) - 1)) / 1609.34
        print(f"DFS Path length (Miles): {dfs_length_miles:.2f}")
        ox.plot_graph_route(G, dfs_path, route_linewidth=2, node_size=0)

    bfs_path = route_algo.bfs_distance(start, goal)
    if bfs_path:
        bfs_length_miles = sum(graph[bfs_path[i]][bfs_path[i + 1]] for i in range(len(bfs_path) - 1)) / 1609.34
        print(f"BFS Path length (Miles): {bfs_length_miles:.2f}")
        ox.plot_graph_route(G, bfs_path, route_linewidth=2, node_size=0)

def main():
    #Get LA graph
    # Load the graph from a GraphML file
    G = ox.load_graphml("./data/manhattan_drive.graphml")
    #print(G.edges(data=True))

    # Need to clean data and train LSTM model
    '''Need to implement this part'''
    
    
    # Euclidean distance as the heuristic function (potentially dont need it)
    graph = Helpers.CreateAdjList(G)

    
    
    #need to get predicted time and update graph
    '''Need to implement this part'''


    # Create an instance of RouteAlgo with the graph
    route_algo = RouteAlgo(graph, G)


    # Define start and end locations
    # We could get user input to make it seem more interative.....
    start_location = "Empire State Building, New York, NY"
    end_location = "Times Square, New York, NY"


    # Get coordinates for each location
    start_coords = Helpers.GetCoords(start_location)
    end_coords = Helpers.GetCoords(end_location)
    

    # Define start and goal nodes
    start = ox.distance.nearest_nodes(G, X=start_coords[1], Y=start_coords[0])
    goal = ox.distance.nearest_nodes(G, X=end_coords[1], Y=end_coords[0])  

    print("Empire State Building Node:", start, "to Times Square Node:", goal)

    print("Start Node:", start, "Goal Node:", goal)

    # Print the path
    #PrintPath(start, goal, graph, G, route_algo)

#Longer distance places in manhattan other than empire state building and times square

    start_location = "Central Park, New York, NY"
    end_location = "Battery Park, New York, NY"

    start_coords = Helpers.GetCoords(start_location)

    end_coords = Helpers.GetCoords(end_location)

    start = ox.distance.nearest_nodes(G, X=start_coords[1], Y=start_coords[0])

    goal = ox.distance.nearest_nodes(G, X=end_coords[1], Y=end_coords[0])

    print("Central Park Node:", start, "to Battery Park Node:", goal)

    print("Start Node:", start, "Goal Node:", goal)

    #PrintPath(start, goal, graph, G, route_algo)

    









if __name__ == "__main__":
    main()


    
    

