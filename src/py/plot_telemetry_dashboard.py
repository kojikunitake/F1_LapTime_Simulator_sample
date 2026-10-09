import os
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

def main():
    # Define input database paths
    output_dir = "../../output_data"
    csv_path = os.path.join(output_dir, "suzuka_optimised_line.csv")
    
    print("=====================================================")
    print("   F1 Pro-Telemetry Integrated Dashboard System      ")
    print("=====================================================")
    
    if not os.path.exists(csv_path):
        print(f"[Error] Target telemetry profile not found: {csv_path}")
        return

    # Load the expanded physics parameters from C++
    print(f"[Loader] Importing advanced telemetry arrays from: {csv_path}")
    df = pd.read_csv(csv_path)

    x = df['x'].values
    y = df['y'].values
    lateral = df['lateral'].values
    v_final = df['v'].values * 3.6  # Convert m/s to km/h
    long_g = df['long_g'].values
    lat_g = df['lat_g'].values
    idx = df['idx'].values
    N = len(x)

    # Reconstruct closed loop smooth coordinates for the track limits representation
    opt_x = np.zeros(N + 1)
    opt_y = np.zeros(N + 1)
    track_half_width = 6.0  # 12m total width definition

    for i in range(N):
        prev_idx = (i - 1 + N) % N
        next_idx = (i + 1) % N
        dx = x[next_idx] - x[prev_idx]
        dy = y[next_idx] - y[prev_idx]
        mag = np.sqrt(dx**2 + dy**2) if (dx**2 + dy**2) > 1e-8 else 1.0
        nx = -dy / mag
        ny = dx / mag
        
        opt_x[i] = x[i] + nx * lateral[i]
        opt_y[i] = y[i] + ny * lateral[i]

    opt_x[N], opt_y[N] = opt_x[0], opt_y[0]

    # Initialise High-Fidelity Stealth Dark Engineering Layout
    fig = plt.figure(figsize=(18, 10), facecolor='#111111')
    gs = gridspec.GridSpec(2, 2, width_ratios=[1.1, 0.9], height_ratios=[1.0, 1.0])
    
    # ---------------------------------------------------------------------------
    # PLOT 1: SUZUKA CIRCUIT SPATIAL TRAJECTORY WITH LATERAL G HEATMAP (LEFT)
    # ---------------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[:, 0])
    ax1.set_facecolor('#151515')
    
    # Plot smooth fading centerline for visual anchoring
    ax1.plot(x, y, color='#333333', linestyle=':', linewidth=1.0, alpha=0.5, label='Centerline')
    
    # Render the racing line mapping color intensity to the computed lateral G-Force
    sc1 = ax1.scatter(opt_x[:-1], opt_y[:-1], c=lat_g, cmap='plasma', s=30, zorder=4, 
                      edgecolor='#111111', linewidth=0.3, label='Optimised Racing Line')
    
    cbar1 = fig.colorbar(sc1, ax=ax1, pad=0.02, aspect=30)
    cbar1.set_label('Absolute Lateral Acceleration (G)', color='white', fontsize=11, labelpad=10)
    cbar1.ax.tick_params(colors='white')
    
    ax1.set_title("Suzuka Circuit - Spatial Trajectory & Lateral G Load", color='white', fontsize=14, fontweight='bold', pad=15)
    ax1.set_xlabel("Spatial X Coordinate (m)", color='#888888', fontsize=10)
    ax1.set_ylabel("Spatial Y Coordinate (m)", color='#888888', fontsize=10)
    ax1.tick_params(colors='#888888', labelsize=9)
    ax1.grid(True, color='#252525', linestyle='-', linewidth=0.5)
    ax1.axis('equal')
    
    legend1 = ax1.legend(facecolor='#222222', edgecolor='#444444', loc='upper right')
    plt.setp(legend1.get_texts(), color='white')

    # ---------------------------------------------------------------------------
    # PLOT 2: G-G DIAGRAM / TRACTION ELLIPSE LIMIT ENVELOPE (TOP RIGHT)
    # ---------------------------------------------------------------------------
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.set_facecolor('#151515')
    
    # Draw standard theoretical F1 physics grip circles (e.g. 5G friction boundaries)
    theta = np.linspace(0, 2*np.pi, 200)
    for r in [2.0, 4.0, 5.0]:
        ax2.plot(r*np.cos(theta), r*np.sin(theta), color='#444444' if r<5.0 else '#ff3355', 
                 linestyle='--' if r<5.0 else '-', linewidth=0.8 if r<5.0 else 1.5, 
                 alpha=0.6, zorder=1)
        ax2.text(0.1, r+0.1, f"{int(r)}G", color='#666666' if r<5.0 else '#ff3355', fontsize=8, alpha=0.7)

    # Scatter actual simulated G-Forces (X: Lateral G, Y: Longitudinal G)
    # Signs match standard telemetry: positive lateral denotes cornering, negative long denotes braking
    sc2 = ax2.scatter(long_g, lat_g, c=v_final, cmap='viridis', s=20, zorder=3, alpha=0.8, edgecolor='none')
    
    cbar2 = fig.colorbar(sc2, ax=ax2, pad=0.02)
    cbar2.set_label('Vehicle Velocity (km/h)', color='white', fontsize=10, labelpad=8)
    cbar2.ax.tick_params(colors='white')

    ax2.set_title("G-G Diagram - Tire Traction Circle Envelope", color='white', fontsize=13, fontweight='bold', pad=12)
    ax2.set_xlabel("Longitudinal Acceleration / Braking (G)", color='#888888', fontsize=10)
    ax2.set_ylabel("Lateral Cornering Forces (G)", color='#888888', fontsize=10)
    ax2.tick_params(colors='#888888', labelsize=9)
    ax2.grid(True, color='#252525', linestyle='-', linewidth=0.5)
    
    # Balance layout symmetrically around the friction bounds (Full 360-degree circle representation)
    ax2.set_xlim(-6.0, 6.0)
    ax2.set_ylim(-6.0, 6.0) # Expanded to show both left and right cornering forces symmetrically
    ax2.set_aspect('equal', 'box') # ADD THIS LINE to guarantee a perfect circle presentation
    ax2.axhline(0, color='#333333', linewidth=1.0)
    ax2.axvline(0, color='#333333', linewidth=1.0)

    # ---------------------------------------------------------------------------
    # PLOT 3: TWO-PASS VELOCITY PROFILE (BOTTOM RIGHT)
    # ---------------------------------------------------------------------------
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.set_facecolor('#151515')
    
    # Extract structural 2-pass reference layers
    v_max = df['vmax'].values * 3.6
    v_fwd = df['vfwd'].values * 3.6
    v_bwd = df['vbwd'].values * 3.6

    ax3.plot(idx, v_max, color='#ff3333', linestyle='--', linewidth=1.0, alpha=0.4, label='v_max (Grip Limit)')
    ax3.plot(idx, v_fwd, color='#ffaa00', linestyle=':', linewidth=1.0, alpha=0.4, label='v_fwd (Traction Pass)')
    ax3.plot(idx, v_bwd, color='#00aaff', linestyle=':', linewidth=1.0, alpha=0.4, label='v_bwd (Braking Pass)')
    ax3.plot(idx, v_final, color='#00ff66', linewidth=2.2, zorder=4, label='v_final (Synthesized)')

    ax3.set_title("Two-Pass Coupled Velocity Profile", color='white', fontsize=13, fontweight='bold', pad=12)
    ax3.set_xlabel("Spatial Node Index (Track Progression)", color='#888888', fontsize=10)
    ax3.set_ylabel("Velocity (km/h)", color='#888888', fontsize=10)
    ax3.tick_params(colors='#888888', labelsize=9)
    ax3.grid(True, color='#252525', linestyle='-', linewidth=0.5)
    ax3.set_xlim(0, N - 1)
    ax3.set_ylim(0, 320)
    
    legend3 = ax3.legend(facecolor='#222222', edgecolor='#444444', loc='lower left', fontsize=9)
    plt.setp(legend3.get_texts(), color='white')

    # Final rendering optimisations
    print("[Graphics] Synthesising layouts and launching interactive window...")
    plt.tight_layout()
    
    # Save the consolidated masterpiece for GitHub README transformation
    dashboard_img = os.path.join(output_dir, "suzuka_telemetry_dashboard.png")
    plt.savefig(dashboard_img, dpi=350, facecolor=fig.get_facecolor(), edgecolor='none')
    print(f"[Graphics] Scalable analytics asset frozen successfully at: {dashboard_img}")
    
    plt.show()

if __name__ == '__main__':
    main()
