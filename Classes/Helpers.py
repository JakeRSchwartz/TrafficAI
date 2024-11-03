from opencage.geocoder import OpenCageGeocode

class Helpers:
    # Initialize the OpenCage geocoder as a class attribute
    api_key = 'b691284543b641248480ea652bbff899'
    geocoder = OpenCageGeocode(api_key)
    
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
    def GetCoords(place_name):
            # Use OpenCage for geocoding
        result = Helpers.geocoder.geocode(place_name)
        if result and len(result):
            return (result[0]['geometry']['lat'], result[0]['geometry']['lng'])
        else:
            raise ValueError(f"Could not geocode location: {place_name}")



    