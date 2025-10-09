import numpy as np
from plyfile import PlyData
import sys

def read_ply(filename):
    """Read PLY file and extract x, y, z, and amplitude data."""
    plydata = PlyData.read(filename)
    vertex = plydata['vertex']
    
    # Extract coordinates
    x = vertex['x']
    y = vertex['y']
    z = vertex['z']
    
    # Try to find amplitude field (could be named differently)
    amplitude_field = None
    for prop in vertex.properties:
        prop_name = prop.name.lower()
        if 'amplitude' in prop_name or 'intensity' in prop_name or 'scalar' in prop_name:
            amplitude_field = prop.name
            break
    
    if amplitude_field is None:
        # If no obvious amplitude field, check all properties
        print("Available properties:", [prop.name for prop in vertex.properties])
        # Try to use the first property after x, y, z
        for prop in vertex.properties:
            if prop.name not in ['x', 'y', 'z']:
                amplitude_field = prop.name
                print(f"Using '{amplitude_field}' as amplitude")
                break
    
    if amplitude_field:
        amplitude = vertex[amplitude_field]
    else:
        raise ValueError("Could not find amplitude field in PLY file")
    
    return np.column_stack([x, y, z, amplitude])

def process_point_cloud(points, dB):
    """
    Process point cloud to remove points with 3dB drop from center maximum.
    
    Args:
        points: Nx4 array with x, y, z, amplitude columns
    
    Returns:
        filtered_points: Filtered point cloud
        center: Center coordinates of filtered cloud
        diameter: Approximate diameter
    """
    # Normalize amplitudes to 0-1 range
    min_amp = np.min(points[:, 3])
    max_amp = np.max(points[:, 3])
    print(f"Original amplitude range: [{min_amp:.6f}, {max_amp:.6f}]")
    
    points[:, 3] = (points[:, 3] - min_amp) / (max_amp - min_amp)
    print(f"Normalized amplitude range: [{np.min(points[:, 3]):.6f}, {np.max(points[:, 3]):.6f}]")
    
    # Find geometric center of the point cloud
    geometric_center = np.mean(points[:, :3], axis=0)
    
    # Calculate distances from geometric center
    distances = np.linalg.norm(points[:, :3] - geometric_center, axis=1)
    
    # Find the 20 centermost points
    center_indices = np.argpartition(distances, 20)[:20]
    center_points = points[center_indices]
    
    print(f"Center 20 points: {len(center_points)}")
    print(f"Center location: {geometric_center}")
    
    # Find maximum amplitude among center points
    max_amplitude_center = np.max(center_points[:, 3])
    print(f"Maximum amplitude in center: {max_amplitude_center:.6f}")
    
    # Calculate 3dB drop threshold
    # 3dB drop in amplitude means the threshold is at: max * 10^(-3/20) = max * 0.7079
    # This is equivalent to: threshold = max - (max * 0.2921)
    db_drop_factor = 10**(-dB/20)  # ≈ 0.7079 (the threshold multiplier)
    drop_amount = max_amplitude_center * (1 - db_drop_factor)  # The absolute drop
    threshold_amplitude = max_amplitude_center - drop_amount  # Same as max * 0.7079
    print(f"Absolute drop from center (3dB): {drop_amount:.6f}")
    print(f"Threshold amplitude: {max_amplitude_center:.6f} - {drop_amount:.6f} = {threshold_amplitude:.6f}")
    
    # Filter points based on amplitude threshold
    filtered_points = points[points[:, 3] >= threshold_amplitude]
    
    print(f"\nOriginal point count: {len(points)}")
    print(f"Filtered point count: {len(filtered_points)}")
    print(f"Points removed: {len(points) - len(filtered_points)}")
    
    # Calculate center of filtered point cloud
    filtered_center = np.mean(filtered_points[:, :3], axis=0)
    
    # Calculate approximate diameter (using maximum distance between points)
    # For efficiency, we'll use the distance from center to furthest point, then double it
    distances_from_center = np.linalg.norm(filtered_points[:, :3] - filtered_center, axis=1)
    max_distance = np.max(distances_from_center)
    diameter = 2 * max_distance
    
    # Alternative: Calculate actual maximum distance between any two points
    # This is more accurate but computationally expensive
    # We'll use a sampling approach for large point clouds
    if len(filtered_points) < 1000:
        # For small clouds, calculate exact diameter
        from scipy.spatial.distance import pdist
        pairwise_distances = pdist(filtered_points[:, :3])
        actual_diameter = np.max(pairwise_distances)
    else:
        # For large clouds, estimate from center
        actual_diameter = diameter
    
    print(f"\nFiltered cloud center: [{filtered_center[0]:.6f}, {filtered_center[1]:.6f}, {filtered_center[2]:.6f}]")
    print(f"Approximate diameter: {actual_diameter:.6f}")
    
    return filtered_points, filtered_center, actual_diameter, max_amplitude_center, threshold_amplitude

def main():
    defect = 'defect4'
    dB = 3
    input_file = (f'/home/rebecca/ut_servoing/src/peak_ros/src/peak_ros/bags/processed_data_220Gain/{defect}_point_cloud.ply')
    
    # Read point cloud
    print("Reading point cloud...")
    points = read_ply(input_file)
    print(f"Loaded {len(points)} points")
    
    # Process point cloud
    print("\nProcessing point cloud...")
    filtered_points, center, diameter, max_amplitude_center, threshold_amplitude = process_point_cloud(points, dB)
    
    # Save results
    output_file = (f'/home/rebecca/ut_servoing/src/peak_ros/src/peak_ros/bags/processed_data_220Gain/{defect}_filtered_point_cloud.txt')
    np.savetxt(output_file, filtered_points, 
               header='x y z amplitude', 
               fmt='%.6f',
               comments='')
    print(f"\nFiltered point cloud saved to: {output_file}")
    
    # Save summary
    summary_file = (f'/home/rebecca/ut_servoing/src/peak_ros/src/peak_ros/bags/processed_data_220Gain/{defect}_analysis_summary.txt')
    with open(summary_file, 'w') as f:
        f.write("Point Cloud Analysis Summary\n")
        f.write("=" * 50 + "\n\n")
        f.write(f"Center coordinates: [{center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f}]\n")
        f.write(f"Approximate diameter: {diameter:.6f}\n")
        f.write(f"\nOriginal points: {len(points)}\n")
        f.write(f"Filtered points: {len(filtered_points)}\n")
        f.write(f"Points removed: {len(points) - len(filtered_points)}\n")
    
    print(f"Summary saved to: {summary_file}")
    
    # Create visualization
    print("\nCreating visualization...")
    create_visualization(points, filtered_points, center, diameter, max_amplitude_center, threshold_amplitude, defect, dB)

def create_visualization(original_points, filtered_points, center, diameter, max_center_amp, threshold, defect, dB):
    """Create comprehensive visualization of the point cloud analysis."""
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    # Extract coordinates and amplitudes
    x_orig = original_points[:, 0]
    y_orig = original_points[:, 1]
    z_orig = original_points[:, 2]
    amp_orig = original_points[:, 3]
    
    x_filt = filtered_points[:, 0]
    y_filt = filtered_points[:, 1]
    z_filt = filtered_points[:, 2]
    amp_filt = filtered_points[:, 3]
    
    # Create figure with multiple subplots
    fig = plt.figure(figsize=(18, 12))
    
    # 1. Original 3D scatter plot
    ax1 = fig.add_subplot(2, 4, 1, projection='3d')
    scatter1 = ax1.scatter(x_orig, y_orig, z_orig, c=amp_orig, cmap='viridis', s=20, alpha=0.6)
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('Original Point Cloud\n(Normalized Amplitude)')
    plt.colorbar(scatter1, ax=ax1, label='Amplitude', shrink=0.5)
    
    # 2. Filtered 3D scatter plot
    ax2 = fig.add_subplot(2, 4, 2, projection='3d')
    scatter2 = ax2.scatter(x_filt, y_filt, z_filt, c=amp_filt, cmap='viridis', s=20, alpha=0.6)
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_title(f'Filtered Point Cloud\n(Above {dB}dB Threshold)')
    plt.colorbar(scatter2, ax=ax2, label='Amplitude', shrink=0.5)
    
    # 3. Amplitude histogram
    ax3 = fig.add_subplot(2, 4, 3)
    ax3.hist(amp_orig, bins=50, edgecolor='black', alpha=0.7, label='Original')
    ax3.axvline(max_center_amp, color='r', linestyle='--', linewidth=2, label=f'Max center: {max_center_amp:.3f}')
    ax3.axvline(threshold, color='orange', linestyle='--', linewidth=2, label=f'{dB}dB threshold: {threshold:.3f}')
    ax3.set_xlabel('Normalized Amplitude')
    ax3.set_ylabel('Count')
    ax3.set_title('Amplitude Distribution')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # 4. Statistics panel
    ax4 = fig.add_subplot(2, 4, 4)
    ax4.axis('off')
    stats_text = f"""Analysis Statistics:
━━━━━━━━━━━━━━━━━━━━━━━━━━
Amplitude (Normalized 0-1):
  Min:     {np.min(amp_orig):.6f}
  Max:     {np.max(amp_orig):.6f}
  Mean:    {np.mean(amp_orig):.6f}
  Median:  {np.median(amp_orig):.6f}

Center Max:  {max_center_amp:.6f}
{dB}dB Threshold: {threshold:.6f}

Point Counts:
  Original:  {len(original_points)}
  Filtered:  {len(filtered_points)}
  Removed:   {len(original_points) - len(filtered_points)}
  (Kept {100*len(filtered_points)/len(original_points):.1f}%)

Filtered Cloud:
  Center: [{center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f}]
  Diameter: {diameter:.6f}
"""
    ax4.text(0.05, 0.5, stats_text, fontsize=9, family='monospace',
             verticalalignment='center', transform=ax4.transAxes)
    
    # 5. XY projection - Original
    ax5 = fig.add_subplot(2, 4, 5)
    scatter5 = ax5.scatter(x_orig, y_orig, c=amp_orig, cmap='viridis', s=30, alpha=0.6)
    ax5.set_xlabel('X')
    ax5.set_ylabel('Y')
    ax5.set_title('XY Projection - Original')
    ax5.axis('equal')
    plt.colorbar(scatter5, ax=ax5, label='Amplitude', shrink=0.8)
    ax5.grid(True, alpha=0.3)
    
    # 6. XY projection - Filtered
    ax6 = fig.add_subplot(2, 4, 6)
    scatter6 = ax6.scatter(x_filt, y_filt, c=amp_filt, cmap='viridis', s=30, alpha=0.6)
    ax6.set_xlabel('X')
    ax6.set_ylabel('Y')
    ax6.set_title('XY Projection - Filtered')
    ax6.axis('equal')
    plt.colorbar(scatter6, ax=ax6, label='Amplitude', shrink=0.8)
    ax6.grid(True, alpha=0.3)
    
    # 7. XZ projection - Original
    ax7 = fig.add_subplot(2, 4, 7)
    scatter7 = ax7.scatter(x_orig, z_orig, c=amp_orig, cmap='viridis', s=30, alpha=0.6)
    ax7.set_xlabel('X')
    ax7.set_ylabel('Z')
    ax7.set_title('XZ Projection - Original')
    ax7.axis('equal')
    plt.colorbar(scatter7, ax=ax7, label='Amplitude', shrink=0.8)
    ax7.grid(True, alpha=0.3)
    
    # 8. XZ projection - Filtered
    ax8 = fig.add_subplot(2, 4, 8)
    scatter8 = ax8.scatter(x_filt, z_filt, c=amp_filt, cmap='viridis', s=30, alpha=0.6)
    ax8.set_xlabel('X')
    ax8.set_ylabel('Z')
    ax8.set_title('XZ Projection - Filtered')
    ax8.axis('equal')
    plt.colorbar(scatter8, ax=ax8, label='Amplitude', shrink=0.8)
    ax8.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save the figure
    output_path = (f'/home/rebecca/ut_servoing/src/peak_ros/src/peak_ros/bags/processed_data_220Gain/{defect}_analysis_visualization.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Visualization saved to: {output_path}")
    plt.close()

if __name__ == "__main__":
    main()