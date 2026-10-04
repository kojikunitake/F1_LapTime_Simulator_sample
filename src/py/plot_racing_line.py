import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

# Resolve the project root directory dynamically (F1_laptime_simulator)
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent

# Set the path to the simulation output file
DATA_FILE = PROJECT_ROOT / "output_data" / "suzuka_optimized_line.csv"

def main():
    if not DATA_FILE.exists():
        print(f"[Error] Execution aborted. Target dataset not found: {DATA_FILE}")
        return

    # Load the optimized trajectory data points
    print(f"[Plotter] Loading optimized racing line from: {DATA_FILE}")
    df = pd.read_csv(DATA_FILE)

    # Plot the 2D optimized racing line trajectory
    plt.figure(figsize=(10, 10))
    plt.plot(df["x"], df["y"], label="Optimized Racing Line", color="blue", linewidth=2)
    
    # Configure graph metadata
    plt.axis("equal")
    plt.xlabel("X Coordinate (m)")
    plt.ylabel("Y Coordinate (m)")
    plt.title("Suzuka Circuit – 2D Optimized Racing Line Trajectory")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    
    print("[Plotter] Rendering 2D spatial trajectory canvas...")
    plt.show()

if __name__ == "__main__":
    main()
