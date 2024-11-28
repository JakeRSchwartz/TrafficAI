from opencage.geocoder import OpenCageGeocode
from config import API_KEY
from classes.TravelTimePredictor import TravelTimePredictor

class Helpers:
    # Initialize the OpenCage geocoder as a class attribute
    geocoder = OpenCageGeocode(API_KEY)
    

    # @staticmethod
    # def CreateAdjList(G):
    #     graph = {}
    #     for node in G.nodes:
    #         graph[node] = {}
    #         for neighbor in G.adj[node]:
    #             edge_data = G.edges[node, neighbor, 0]  # Edge data
    #             graph[node][neighbor] = edge_data['length']  # Distance between nodes
    #     return graph

    @staticmethod
    def CreateAdjList(G):
        """
        Create an adjacency list with distance and travel time for each edge.
        Handles cases where 'maxspeed' might be a list or missing.
        """
        graph = {}

        for node in G.nodes:
            graph[node] = {}
            for neighbor in G.adj[node]:
                # Retrieve edge data (assumes first edge, key=0, in MultiDiGraph)
                edge_data = G.edges[node, neighbor, 0]

                # Extract length
                length = edge_data.get('length', 0)  # Length in meters

                # Extract and process maxspeed
                maxspeed_data = edge_data.get('maxspeed', '25 mph')  # Default to 25 mph
                maxspeed = 25  # Default value

                try:
                    if isinstance(maxspeed_data, list):
                        # Clean up list elements and convert to integers
                        maxspeed_values = [
                            int(speed.strip().split()[0]) for speed in maxspeed_data
                        ]
                        maxspeed = min(maxspeed_values)  # Use the lowest speed
                    elif isinstance(maxspeed_data, str):
                        # Process single string value
                        maxspeed = int(maxspeed_data.strip().split()[0])
                except (ValueError, IndexError, AttributeError) as e:
                    print(f"Error parsing maxspeed for edge ({node} -> {neighbor}): {maxspeed_data}")
                    maxspeed = 25  # Default to 25 mph if parsing fails

                # Convert maxspeed to m/s
                maxspeed_mps = maxspeed * 0.44704

                # Calculate travel time in seconds
                travel_time_seconds = length / maxspeed_mps if maxspeed_mps > 0 else float('inf')

                # Ensure all edges are stored as dictionaries
                graph[node][neighbor] = {
                    'length': length,                   # Distance in meters
                    'travel_time_seconds': travel_time_seconds,  # Travel time in seconds
                    'maxspeed': maxspeed
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
        



    