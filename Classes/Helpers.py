class Helpers:
    @staticmethod
    def CreateAdjList(G):
        graph = {}
        for node in G.nodes:
            graph[node] = {}
            for neighbor in G.adj[node]:
                edge_data = G.edges[node, neighbor, 0]  # Edge data (with `length` attribute)
                graph[node][neighbor] = edge_data['length']  # Distance between nodes
        return graph


    