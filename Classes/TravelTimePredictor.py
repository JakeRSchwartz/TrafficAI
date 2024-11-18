import torch
class TravelTimePredictor:
    def __init__(self, model):
        self.model = model
        self.model.eval()  # Set the model to evaluation mode

    def predict_travel_time(self, edge_data):
        with torch.no_grad():  # Disable gradient calculation for inference
            edge_tensor = torch.tensor(edge_data, dtype=torch.float).unsqueeze(0)  # Add batch dimension
            travel_time = self.model(edge_tensor).item()
        return travel_time