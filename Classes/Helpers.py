from opencage.geocoder import OpenCageGeocode
from config import API_KEY
from classes.TravelTimePredictor import TravelTimePredictor

class Helpers:
    # Initialize the OpenCage geocoder as a class attribute
    geocoder = OpenCageGeocode(API_KEY)
    
    # Create an generic adjacency list from a graph with NO times.
    @staticmethod
    def CreateAdjList(G):
        graph = {}
        for node in G.nodes:
            graph[node] = {}
            for neighbor in G.adj[node]:
                edge_data = G.edges[node, neighbor, 0]  # Edge data
                graph[node][neighbor] = edge_data['length']  # Distance between nodes
        return graph
    
    @staticmethod
    def UpdateGraphWithPredictions(G, model):
        graph = {}
        for node in G.nodes:
            graph[node] = {}
            for neighbor in G.adj[node]:
                edge_data = G.edges[node, neighbor, 0]
                
                # Assume edge_data contains 'features' key with necessary inputs for LSTM
                edge_features = edge_data['features']  # Relevant features for prediction
                predicted_time = TravelTimePredictor.predict_travel_time(edge_features, model)
                
                # Store both distance and predicted travel time for each edge
                graph[node][neighbor] = {
                    'distance': edge_data['length'],  # Original distance data
                    'predicted_time': predicted_time  # Predicted travel time
                }
                
        return graph

    
    # Get Coordinates
    @staticmethod
    def GetCoords(place_name):
            # Use OpenCage for geocoding
        result = Helpers.geocoder.geocode(place_name)
        if result and len(result):
            return (result[0]['geometry']['lat'], result[0]['geometry']['lng'])
        else:
            raise ValueError(f"Could not geocode location: {place_name}")
        



    