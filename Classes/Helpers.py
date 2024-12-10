from opencage.geocoder import OpenCageGeocode
from config import API_KEY

class Helpers:
    # Initialize the OpenCage geocoder as a class attribute
    geocoder = OpenCageGeocode(API_KEY)

    @staticmethod
    def CreateAdjList(G):
        graph = {}

        for node in G.nodes:
            graph[node] = {}
            for neighbor in G.adj[node]:
                edge_data = G.edges[node, neighbor, 0]

                # Extract length
                length = edge_data.get('length', 0)  # Length in meters

                #Extract maxspeed
                maxspeed_data = edge_data.get('maxspeed', '25 mph')  
                maxspeed = 25  # Default 

                try:
                    if isinstance(maxspeed_data, list):
                        # Clean up list elements and convert to integers
                        maxspeed_values = [
                            int(speed.strip().split()[0]) for speed in maxspeed_data
                        ]
                        maxspeed = min(maxspeed_values)
                    elif isinstance(maxspeed_data, str):
                        maxspeed = int(maxspeed_data.strip().split()[0])
                except (ValueError, IndexError, AttributeError) as e:
                    print(f"Error parsing maxspeed for edge ({node} -> {neighbor}): {maxspeed_data}")
                    maxspeed = 25  # Default to 25 mph

                # Convert to m/s
                maxspeed_mps = maxspeed * 0.44704

                # Calculate travel time in seconds
                travel_time_seconds = length / maxspeed_mps if maxspeed_mps > 0 else float('inf')

                graph[node][neighbor] = {
                    'length': length,                   
                    'travel_time_seconds': travel_time_seconds,  
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
        



    