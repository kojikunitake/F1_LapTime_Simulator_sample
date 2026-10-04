import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

# Resolve the project root directory dynamically (F1_laptime_simulator)
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent

# Set the path to the dense telemetry log file
LOG_FILE = PROJECT_ROOT / "output_data" / "logs" / "vmax_log.csv"

def main():
    if not LOG_FILE.exists():
        print(f"[Error] Execution aborted. Telemetry log not found: {LOG_FILE}")
        return

    # Load the dynamic velocity profile log array
    print(f"[Plotter] Loading dense telemetry streaming dataset from: {LOG_FILE}")
    df = pd.read_csv(LOG_FILE)

    # Initialize a wide canvas profile for analytical resolution
    plt.figure(figsize=(12, 6))
    
    # Plot target velocity tracking streams
    plt.plot(df["s"], df["vmax"], label="Dynamic Grip Velocity Cap: vmax (m/s)", color="red", alpha=0.8)
    plt.plot(df["s"], df["v"], label="Vehicle Instantaneous Speed: v (m/s)", color="blue", alpha=0.8)
    plt.plot(df["s"], df["vmax"] - df["v"], label="Velocity Tracking Error Delta: vmax - v (m/s)", color="purple", linestyle="--", alpha=0.5)

    # Configure graph metadata
    plt.xlabel("Absolute Cumulative Distance Along Centerline (m)")
    plt.ylabel("Velocity Profile Output (m/s)")
    plt.title("Suzuka Circuit Telemetry Analysis – Dynamic Speed vs Grip Cap")
    plt.grid(True, linestyle=":", alpha=0.6)
    plt.legend(loc="upper right")
    
    print("[Plotter] Rendering dynamic velocity telemetry map...")
    plt.show()

if __name__ == "__main__":
    main()
