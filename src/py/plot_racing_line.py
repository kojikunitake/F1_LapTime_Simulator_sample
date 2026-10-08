import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def main():
    # Define telemetry paths
    output_dir = "../../output_data"
    csv_path = os.path.join(output_dir, "suzuka_optimised_line.csv")
    
    print("=====================================================")
    print("      F1 Trajectory Visualisation Subsystem          ")
    print("=====================================================")
    
    if not os.path.exists(csv_path):
        print(f"[Error] Target telemetry file not found: {csv_path}")
        return

    # Load the spatial profile metrics exported by C++
    print(f"[Loader] Importing telemetry database from: {csv_path}")
    df = pd.read_csv(csv_path)

    x = df['x'].values
    y = df['y'].values
    lateral = df['lateral'].values
    N = len(x)

    # Pre-allocate closed arrays for smooth plotting (joining the final node to the start)
    opt_x = np.zeros(N + 1)
    opt_y = np.zeros(N + 1)
    left_x, left_y = np.zeros(N + 1), np.zeros(N + 1)
    right_x, right_y = np.zeros(N + 1), np.zeros(N + 1)
    
    # Virtual track width configuration for Suzuka representation (e.g., 12m total track width)
    track_half_width = 6.0 

    # Compute high-fidelity smooth normal vectors treating the circuit as a closed loop
    for i in range(N):
        # Cyclical index mapping for seamless forward/backward lookup
        prev_idx = (i - 1 + N) % N
        next_idx = (i + 1) % N
        
        dx = x[next_idx] - x[prev_idx]
        dy = y[next_idx] - y[prev_idx]
            
        mag = np.sqrt(dx**2 + dy**2)
        if mag < 1e-5:
            mag = 1.0
            
        nx = -dy / mag  # Standard 2D Normal vector pointing inwards/outwards
        ny = dx / mag   

        # Reconstruct coordinates safely using the updated physics vector coefficients
        opt_x[i] = x[i] + nx * lateral[i]
        opt_y[i] = y[i] + ny * lateral[i]
        
        left_x[i]  = x[i] + nx * track_half_width
        left_y[i]  = y[i] + ny * track_half_width
        right_x[i] = x[i] - nx * track_half_width
        right_y[i] = y[i] - ny * track_half_width

    # Explicitly close the cyclical loop graphics pipeline
    opt_x[N], opt_y[N] = opt_x[0], opt_y[0]
    left_x[N], left_y[N] = left_x[0], left_y[0]
    right_x[N], right_y[N] = right_x[0], right_y[0]

    # Set up Dark Stealth theme for F1 dashboard rendering
    plt.figure(figsize=(14, 11), facecolor='#111111')
    ax = plt.axes()
    ax.set_facecolor('#161616')

    # 1. Plot Restructured Track Boundaries (Beautiful continuous asphalt borders)
    plt.plot(left_x, left_y, color='#555555', linestyle='-', linewidth=1.8, alpha=0.9, label='Track Limits')
    plt.plot(right_x, right_y, color='#555555', linestyle='-', linewidth=1.8, alpha=0.9)

    # 2. Plot Reference Centerline (Baseline)
    plt.plot(np.append(x, x[0]), np.append(y, y[0]), color='#0055ff', linestyle=':', linewidth=1.2, alpha=0.5, label='Centerline (Baseline)')

    # 3. Plot the mathematical masterpiece: The 2-Pass Optimised Racing Line
    plt.plot(opt_x, opt_y, color='#00ff66', linewidth=3.0, zorder=4, label='Optimised Racing Line (Ideal)')
    
    # Add scatter color-mesh to show spatial sequence points clearly without overlapping text
    sc = plt.scatter(opt_x[:-1], opt_y[:-1], c=df['idx'], cmap='plasma', s=25, zorder=5, alpha=0.9, edgecolor='#111111', linewidth=0.5)
    cbar = plt.colorbar(sc, ax=ax, pad=0.02)
    cbar.set_label('Track Point Index (Spatial Flow)', color='white', fontsize=11, labelpad=10)
    cbar.ax.tick_params(colors='white')

    # Styling and cosmetics
    plt.title("Suzuka Circuit - 2-Pass Optimal Trajectory Resolution", color='white', fontsize=16, fontweight='bold', pad=25)
    plt.xlabel("Spatial X Coordinate (m)", color='white', fontsize=12)
    plt.ylabel("Spatial Y Coordinate (m)", color='white', fontsize=12)
    
    ax.tick_params(colors='white', labelsize=10)
    ax.grid(True, color='#282828', linestyle='-', linewidth=0.5)
    
    # CRITICAL: Enforce strict equal aspect ratio to completely eradicate track warping
    plt.axis('equal')
    
    legend = plt.legend(facecolor='#222222', edgecolor='#444444', loc='upper right', fontsize=11)
    for text in legend.get_texts():
        text.set_color('white')

    print("[Graphics] Generating layout render engine window...")
    plt.tight_layout()
    
    # Save a high-resolution PNG for GitHub repository asset documentation
    output_img = os.path.join(output_dir, "suzuka_racing_line.png")
    plt.savefig(output_img, dpi=300, facecolor=plt.gcf().get_facecolor(), edgecolor='none')
    print(f"[Graphics] Scalable vector preview snapshot frozen at: {output_img}")
    
    plt.show()

if __name__ == '__main__':
    main()
