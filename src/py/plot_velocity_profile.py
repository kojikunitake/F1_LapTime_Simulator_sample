import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def main():
    # Define telemetry paths
    output_dir = "../../output_data"
    csv_path = os.path.join(output_dir, "suzuka_optimised_line.csv")
    
    print("=====================================================")
    print("      F1 Velocity Profile Analytics Subsystem        ")
    print("=====================================================")
    
    if not os.path.exists(csv_path):
        print(f"[Error] Target telemetry file not found: {csv_path}")
        print("Please ensure your C++ engine has exported the velocity vectors.")
        return

    # Load the spatial profile metrics
    print(f"[Loader] Importing velocity database from: {csv_path}")
    df = pd.read_csv(csv_path)

    # Convert node indices to a continuous X-axis representation
    idx = df['idx'].values
    
    # Set up Dark Stealth theme for F1 engineering charts
    plt.figure(figsize=(14, 8), facecolor='#111111')
    ax = plt.axes()
    ax.set_facecolor('#161616')

    # Dynamic Column Mapping with Fallbacks
    # standard columns: vmax, vfwd, vbwd, vfinal (or similar)
    has_profiles = all(col in df.columns for col in ['vmax', 'vfwd', 'vbwd', 'v'])
    
    if has_profiles:
        # Convert m/s to km/h for intuitive F1 telemetry presentation
        v_max = df['vmax'].values * 3.6
        v_fwd = df['vfwd'].values * 3.6
        v_bwd = df['vbwd'].values * 3.6
        v_final = df['v'].values * 3.6

        # 1. Plot individual pass boundaries (Light dashed tracks)
        plt.plot(idx, v_max, color='#ff3333', linestyle='--', linewidth=1.2, alpha=0.5, label='G-Grip Curvature Limit (v_max)')
        plt.plot(idx, v_fwd, color='#ffaa00', linestyle=':', linewidth=1.2, alpha=0.5, label='Forward Traction Limit (v_fwd)')
        plt.plot(idx, v_bwd, color='#00aaff', linestyle=':', linewidth=1.2, alpha=0.5, label='Backward Braking Limit (v_bwd)')
        
        # 2. Plot the Synthesized Master Envelope (Solid Line)
        plt.plot(idx, v_final, color='#00ff66', linewidth=2.5, zorder=4, label='Synthesized Vehicle Velocity (v_final)')
        
        # Highlight strict braking points (where backward pass matches final profile)
        braking_zones = np.where((np.abs(v_final - v_bwd) < 0.1) & (v_final < v_fwd))[0]
        if len(braking_zones) > 0:
            plt.scatter(idx[braking_zones], v_final[braking_zones], color='#ff0055', s=10, zorder=5, label='Active Braking Zones')

    else:
        # Fallback if the CSV currently only contains geometric data (idx, x, y, lateral)
        print("[Warning] Detailed 2-pass velocity profiles (vmax, vfwd, vbwd) not detected in CSV.")
        print("[Simulate] Generating analytical velocity profile for visual pipeline validation...")
        
        # Virtual placeholder wave showing deceleration into apexes for layout representation
        simulated_v = 240.0 + 50.0 * np.sin(idx * 0.15) - 30.0 * np.cos(idx * 0.05)
        plt.plot(idx, simulated_v, color='#00ff66', linewidth=2.5, label='Velocity Envelope (km/h)')
        plt.title("Suzuka Circuit - Velocity Profile Pipeline [Simulation Mode]", color='white', fontsize=15, fontweight='bold', pad=25)

    # Chart Cosmetics
    if has_profiles:
        plt.title("Suzuka Circuit - Two-Pass Forward-Backward Velocity Profile", color='white', fontsize=15, fontweight='bold', pad=25)
    
    plt.xlabel("Spatial Node Index (Track Progression)", color='white', fontsize=12)
    plt.ylabel("Vehicle Velocity (km/h)", color='white', fontsize=12)
    
    ax.tick_params(colors='white', labelsize=10)
    ax.grid(True, color='#282828', linestyle='-', linewidth=0.5)
    
    plt.xlim(0, len(idx) - 1)
    plt.ylim(0, 320) # F1 velocity ceiling bound
    
    legend = plt.legend(facecolor='#222222', edgecolor='#444444', loc='lower left', fontsize=11)
    for text in legend.get_texts():
        text.set_color('white')

    print("[Graphics] Generating velocity curve layout view...")
    plt.tight_layout()
    
    # Save a high-resolution PNG asset for documentation expansion
    output_img = os.path.join(output_dir, "suzuka_velocity_profile.png")
    plt.savefig(output_img, dpi=300, facecolor=plt.gcf().get_facecolor(), edgecolor='none')
    print(f"[Graphics] Profile plot frozen successfully at: {output_img}")
    
    plt.show()

if __name__ == '__main__':
    main()
