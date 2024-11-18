import torch
import torch.nn as nn
import torch.optim as optim

class TravelTimeLSTM(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super(TravelTimeLSTM, self).__init__()
        self.lstm = nn.LSTM(input_size, hidden_size, batch_first=True)
        self.fc = nn.Linear(hidden_size, output_size)
    
    def forward(self, x):
        out, _ = self.lstm(x.unsqueeze(1))  # Adding sequence dimension
        out = self.fc(out[:, -1, :])  # Use the last output of the LSTM
        return out
    
