# 🚦 Traffic AI: Pathfinding and LSTM for Traffic Congestion

Welcome to **Traffic AI**, where traditional pathfinding meets advanced machine learning! This project is designed to optimize routes and predict traffic congestion, offering a smarter and more efficient travel solution. 🌐✨

---

## 🌟 **Features**
- **Pathfinding Algorithms**: A*, Dijkstra, BFS, and DFS compute shortest and fastest routes.
- **Predictive Models**: RNN and LSTM forecast vehicle congestion patterns.
- **Hybrid Solution**: Combines static algorithms with dynamic traffic predictions.
- **Real Data**: Manhattan road network from OpenStreetMap and vehicle count data from Kaggle.

---

## 📌 **Objectives**
1. **Optimize Routes**: Find efficient paths using graph-based methods.
2. **Predict Traffic**: Anticipate congestion with machine learning.
3. **Bridge Static and Dynamic**: Seamlessly integrate traditional algorithms and predictive insights.

---

## 🛠️ **Tech Stack**
- **Programming Language**: Python 🐍
- **Libraries**:
  - `osmnx`: For road network graph construction.
  - `networkx`: For pathfinding algorithm implementation.
  - `numpy`, `pandas`: Data manipulation and preprocessing.
  - `tensorflow`, `keras`: RNN and LSTM modeling.
  - `matplotlib`: Data visualization.
- **Data Sources**:
  - OpenStreetMap (Manhattan road network).
  - Kaggle (time-series vehicle count dataset).

---

## 📊 **Data Overview**
### Pathfinding Data:
- Graph attributes:
  - **Length**: Road segment distances (meters).
  - **Maxspeed**: Speed limits (m/s).
  - **Travel Time**: Calculated for each segment (seconds).
- Preprocessed into an adjacency list for efficient route calculations.

### Machine Learning Data:
- **Features**:
  - Vehicle counts (15-minute intervals).
  - Day-of-week and traffic situation (one-hot encoded).
  - Lag features (up to 3 days of historical data).
  - Sine/Cosine encoding for cyclical time features.
- Temporal patterns enable predictive models to capture trends and dependencies.

---

## 📚 **Algorithms**
### Pathfinding:
- **A***: Fast and heuristic-driven.
- **Dijkstra**: Guaranteed shortest path, all nodes explored.
- **BFS/DFS**: Fundamental graph traversal techniques.

### Machine Learning:
- **RNN**: Handles sequential traffic data.
- **LSTM**: Excels in capturing long-term dependencies for time-series predictions.

---

## 🚀 **Getting Started**
1. **Clone the Repository**:
   ```bash      
   git clone https://github.com/yourusername/TrafficAI.git
   cd TrafficAI
   ```
   ## 📜 **Sources**

### 🗺️ **Data Sources**
- **OpenStreetMap**: Road network data for Manhattan, forming the backbone of our pathfinding graph.
- **Kaggle**: Time-series dataset for vehicle counts, used for traffic condition prediction.

### 📚 **Libraries**
- **osmnx**: Graph creation and road network preprocessing.
- **networkx**: Pathfinding algorithm implementation.
- **tensorflow/keras**: RNN and LSTM modeling.
- **numpy/pandas**: Data manipulation and preprocessing.
- **matplotlib**: Visualization of traffic trends and predictions.
  
   
   

