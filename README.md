# F1 Laptime Simulator & Racing Line Optimizer

An advanced, high-performance **F1 Laptime Simulator and Trajectory Optimizer** built using **modern C++ (C++17)** for the core physics pipeline and **Python** for visualization and data processing. 

The engine simulates Formula 1 vehicle dynamics by coupling multi-sector longitudinal acceleration solvers with a reactive lateral grip constraint framework, optimizing racing lines via numerical gradient descent.

---

## 🚀 Key Technical Features

- **High-Performance C++17 Core Engine:** Implements a decoupled structure (Headers/Sources) ensuring fast build cycles, explicit memory safety, and robust file I/O operations using `<filesystem>`.
- **Decoupled Vehicle Dynamics Model:**
  - *Longitudinal Model:* Integrates dynamic propulsion forces scaled by throttle inputs balanced against speed-dependent aerodynamic drag (\(F_{drag} = \frac{1}{2} \rho A C_d v^2\)) and heavy braking deceleration vectors.
  - *Lateral Grip Constraint Solver:* Models tire adhesion limits (up to 5G lateral acceleration) by calculating local curve radius and track curvature (κ) from dynamic 3-point telemetry vectors.
- **Racing Line Optimization:** Utilizes an iterative **Numerical Gradient Descent (Gradient Mutation)** algorithm to transition trajectories from the basic track centerline to the idealized, time-optimal racing profile while respecting physical track-width boundaries.
- **Conditioned Telemetry Streaming:** Features an analytical disk-I/O optimization toggle (`debug_mode`) to record dense telemetry logs (`vmax_log.csv`) on demand without sacrificing core computational throughput during multi-epoch optimization cycles.

---

## 📂 Project Architecture

```text
F1_laptime_simulator/
│
├── src/                      # Source Code Base
│   ├── cpp/                  # Core Simulation Pipeline (Modern C++)
│   │   ├── main.cpp          # Execution cycle and environment management
│   │   ├── simulator.cpp     # Physics solvers and Gradient Descent optimization
│   │   └── simulator.hpp     # Class structure, type specs, and object definition
│   └── py/                   # Python Utilities (Telemetry Analysis & Plotting)
│
├── input_data/               # Source Geometries
│   └── circuits/             # Target track telemetry specifications (JSON format)
│       └── suzuka_track.json 
│
└── output_data/              # Simulation Deliverables
    └── logs/                 # Real-time streamed step logs (CSV telemetry outputs)
        └── vmax_log.csv
```

---

## 🛠️ Build and Execution Guide

### Prerequisites
- A C++17 compliant compiler (GCC 8+, Clang 7+, or MSVC 2017+)
- **nlohmann/json** (Modern C++ JSON library)

> 💡 **Note on External Dependency:** The simulation engine utilizes `nlohmann/json` to parse track profiles. To maintain a clean repository footprint, the header file is not bundled. Please ensure it is installed on your system using one of the methods below.

#### Installing nlohmann/json
Depending on your operating system and environment, you can install the dependency via your preferred package manager:

- **Windows (vcpkg):**
  ```bash
  vcpkg install nlohmann-json
  ```
- **macOS (Homebrew):**
  ```bash
  brew install nlohmann-json
  ```
- **Linux (Ubuntu/Debian):**
  ```bash
  sudo apt-get install nlohmann-json3-dev
  ```
*Alternatively, you can simply drop the `json.hpp` header directly into your local header search path or your compiler's include directory.*

### Compiling with MSVC (Windows / VSCode)
Configure your `.vscode/tasks.json` compiler flags array to enforce C++17 mode (`/std:c++17`) and compile all implementation sources simultaneously:

```bash
cl.exe /Zi /EHsc /nologo /std:c++17 /Fe:main.exe *.cpp
```

### Compiling with GCC/Clang (Linux / Mac)
```bash
g++ -std=c++17 main.cpp simulator.cpp -o f1_simulator
```

### Running the Python Visualization Tools
First, install the required dependencies using the provided `requirements.txt`:
```bash
pip install -r requirements.txt
```

Then, execute the plotting scripts to view telemetry graphs:
```bash
python src/py/plot_racing_line.py
python src/py/plot_velocity_profile.py
```

---

## 📊 Telemetry Output Specifications

When `debug_mode` is enabled, the calculation engine streams dense runtime data arrays capturing physical state transitions across 20,000 steps per lap:

| Field | Description | Type |
|---|---|---|
| `idx` | Discrete spatial coordinate track point index | Integer |
| `s` | Absolute cumulative distance traveled along track centerline (meters) | Double |
| `kappa` | Local track curvature (\(\kappa = \frac{4 \cdot \text{Area}}{a \cdot b \cdot c}\)) | Double |
| `vmax` | Aero-G grip bounded absolute maximum safe velocity (m/s) | Double |
| `v` | Instantaneous physical vehicle velocity profile output (m/s) | Double |

---

## 📈 Future Roadmap & Algorithm Enhancements

To further advance the simulator's fidelity and transition from a reactive model to a highly predictive racing engineer tool, the following enhancements are planned:

### 1. Implementation of the Forward-Backward Simulation Method
The current velocity solver operates reactively based on local track conditions. To properly simulate perfect racing lines, a **Two-Pass Forward-Backward Velocity Solver** will be integrated:
- **Forward Pass:** Computes maximum acceleration capability from the corner exit bounding engine power and traction limits.
- **Backward Pass:** Integrates braking deceleration limits in reverse from the corner apex to accurately determine the exact theoretical **Braking Point** on the straight.
- *Impact:* This eliminates sudden brake-stumping spikes and establishes an idealized velocity profile identical to professional laptime simulation software (e.g., LapSim).

### 2. Predictive Preview Driver Model (Optimal Control)
- Replace the current spatial reactive control loop with a **Look-Ahead Driver Model** (using Predictive Preview Distance). 
- This will simulate human/driver visual perception, allowing the vehicle model to pre-align inputs (smooth steering entry) prior to entering sharp curvature transitions.

### 3. Dynamic Tire Thermal & Degradation Curves
- Integrate dynamic friction coefficient equations dependent on slip angle, wheel load, and compound surface temperature (modelling Pirelli's soft/medium/hard operating windows).

### 4. Disk I/O & Computation Optimization
- Implement an in-memory ring-buffer telemetry logger for the C++ core to further eliminate remaining disk-I/O overhead during dense multi-epoch optimization cycles.
