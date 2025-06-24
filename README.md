
# ⭐ PotentialStar-Algorithm-Benchmark

A visual benchmarking framework to evaluate a custom hybrid path planning algorithm called **Potential\*** — a fusion of **Potential Fields** with fallback **A\*** escape logic. This interactive simulator enables side-by-side comparison of A\*, BFS, and Potential\* in grid environments using real-time PyGame visualization.

---

## 💡 What is Potential\*?

**Potential\*** is a hybrid pathfinding algorithm designed to efficiently reach a goal while avoiding obstacles. It combines:

- **Potential Field navigation**: Computes attractive forces toward the goal and repulsive forces away from nearby walls.
- **A\* fallback**: When the potential field gets "stuck" (in a local minimum or blocked zone), A\* takes over with a limited node budget (e.g., 500 nodes) to escape.
- Once escape is successful, the algorithm returns to potential-based movement.

> ✅ This allows Potential\* to intelligently blend fast reactive movement with guaranteed escape from traps.

---

## 📌 Key Features

- 🚀 **Custom Hybrid Algorithm** (Potential\*) that dynamically switches modes
- 🔁 **Algorithm Switching** between A\*, BFS, and Potential\*
- 🎨 **Live PyGame Grid Visualization**
- 📊 **Built-in Performance Statistics**
- 💾 **Grid Save/Load Functionality**
- 📋 **Side Panel UI with Algorithm Info**

---

## 📁 Repository Contents

| File | Description |
|------|-------------|
| `AlgoBenchmark.py` | Full visualizer and pathfinding logic (self-contained) |
| `PathPlanning_Documentation.pdf` | Deep technical explanation of the implementation, algorithms, grid logic, and benchmarks |

---

## ▶️ Getting Started

### 1. Install dependencies

Ensure Python 3 is installed. Then run:

```bash
pip install -r requirements.txt
```

### 2. Launch the simulation

```bash
python AlgoBenchmark.py
```

---

## 🎮 Controls

| Input | Action |
|-------|--------|
| **Left Click** | Place Start → Goal → Walls (in order) |
| **Right Click** | Remove cell (reset to empty) |
| `TAB` | Switch algorithm (A\*, BFS, Potential\*) |
| `SPACE` | Run selected algorithm |
| `R` | Reset the grid completely |
| `M` | Generate a random obstacle map |
| `S` | Save the current map |
| `L` | Load a saved map |
| `F` | Toggle fast mode (skip animation for speed)

---

## 🤖 Algorithms Overview

### 🔷 A\* (A-Star)
- Uses Manhattan heuristic
- Guarantees shortest path
- Searches intelligently toward goal
- May explore wide area in dense maps

### 🔶 BFS (Breadth-First Search)
- Explores in expanding layers (uniform cost)
- Guarantees shortest path
- Slower and explores more nodes
- No heuristic guidance

### 🌟 Potential\* (Custom Hybrid)
- Starts with **Potential Field** logic:
  - Pulls toward the goal
  - Pushes away from nearby walls
- Detects stagnation or loops
- Switches to **A\*** with a 500-node limit to escape
- Returns to potential field once unstuck
- Final path is **seamlessly stitched** for smooth visualization

---

## 📊 Example Performance

| Algorithm    | Explored Nodes | Path Length | Time (ms) | Notes                      |
|--------------|----------------|-------------|-----------|----------------------------|
| A\*          | ~1,200         | Optimal     | ~45 ms    | Heuristic-based search     |
| BFS          | ~3,500         | Optimal     | ~120 ms   | Wide exploration           |
| Potential\*  | ~400           | ~105%       | ~35 ms    | Efficient with fallback    |

---

## 📄 Documentation

> For full code walk-throughs, algorithm deep dives, and system architecture:

📎 Open **`PathPlanning_Documentation.pdf`**

Includes:
- 📌 Grid generation and pixel logic
- 🧠 Force field calculations and heuristics
- 📈 Algorithm design decisions and performance metrics
- 🧭 Step-by-step pseudocode-style explanations

---

## 🛠️ Future Ideas

- Add **dynamic obstacles** and moving agents
- Support **diagonal movement** and weighted terrains
- Extend to **3D grid environments**
- Add a **GUI toggle for visual options**

## 🙌 Acknowledgments

Special thanks to team members and RCS. This project is inspired by a blend of academic robotics theory and interactive design.
