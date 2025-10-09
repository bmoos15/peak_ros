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

def fit_plane_pca(points):
    """
    Fit a plane to the point cloud using PCA.
    Returns the plane normal and two orthogonal basis vectors in the plane.
    """
    # Get XYZ coordinates only
    xyz = points[:, :3]
    
    # Center the points
    centroid = np.mean(xyz, axis=0)
    centered = xyz - centroid
    
    # Perform PCA
    covariance = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eig(covariance)
    
    # Sort by eigenvalues (descending)
    idx = eigenvalues.argsort()[::-1]
    eigenvalues = eigenvalues[idx]
    eigenvectors = eigenvectors[:, idx]
    
    # The normal to the plane is the eigenvector with smallest eigenvalue
    normal = eigenvectors[:, 2]
    
    # The two principal directions in the plane
    plane_basis_1 = eigenvectors[:, 0]  # Primary direction in plane
    plane_basis_2 = eigenvectors[:, 1]  # Secondary direction in plane
    
    print(f"\nPlane fitting (PCA):")
    print(f"Centroid: {centroid}")
    print(f"Normal vector: {normal}")
    print(f"Eigenvalues: {eigenvalues}")
    print(f"Plane basis 1: {plane_basis_1}")
    print(f"Plane basis 2: {plane_basis_2}")
    
    return normal, plane_basis_1, plane_basis_2, centroid

def project_to_plane_coords(points, plane_basis_1, plane_basis_2, normal, centroid):
    """
    Project points to 2D coordinates using the plane basis vectors.
    Returns coordinates in the plane and perpendicular to the plane.
    """
    xyz = points[:, :3]
    centered = xyz - centroid
    
    # Project onto basis vectors
    u = np.dot(centered, plane_basis_1)  # Coordinate along first plane direction
    v = np.dot(centered, plane_basis_2)  # Coordinate along second plane direction
    w = np.dot(centered, normal)          # Distance from plane (perpendicular)
    
    return u, v, w

def process_point_cloud(points, dB):
    """
    Process point cloud to remove points with 3dB drop from maximum amplitude point.
    
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
    
    # Find point with maximum amplitude
    max_amp_index = np.argmax(points[:, 3])
    max_amp_point = points[max_amp_index]
    max_amplitude_center = max_amp_point[3]
    
    print(f"Maximum amplitude point: [{max_amp_point[0]:.6f}, {max_amp_point[1]:.6f}, {max_amp_point[2]:.6f}]")
    print(f"Maximum amplitude value: {max_amplitude_center:.6f}")
    
    # Calculate 3dB drop threshold
    # 3dB drop in amplitude means the threshold is at: max * 10^(-3/20) = max * 0.7079
    # This is equivalent to: threshold = max - (max * 0.2921)
    db_drop_factor = 10**(-dB/20)  # ≈ 0.7079 (the threshold multiplier)
    drop_amount = max_amplitude_center * (1 - db_drop_factor)  # The absolute drop
    threshold_amplitude = max_amplitude_center - drop_amount  # Same as max * 0.7079
    print(f"Absolute drop from maximum ({dB}dB): {drop_amount:.6f}")
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
    dB = 3
    
    # List of all defects to process
    defects = ['defect6_adjusted_max']
    # defects = ['Defect_1', 'Defect_2', 'Defect_3', 'Defect_4', 'Defect_5', 
    #            'Defect_6', 'Defect_7', 'Defect_8', 'Defect_9', 'Defect_10',
    #            'defect_4', 'defect_5', 'defect_6', 'defect_7', 'defect_8']
    
    base_path = '/home/rebecca/ut_servoing/src/peak_ros/src/peak_ros/bags/processed_data_220Gain/Defects_2/'
    
    # Open consolidated summary file
    consolidated_summary_file = f'{base_path}all_defects_analysis_summary_maxcenter.txt'
    
    with open(consolidated_summary_file, 'w') as summary_f:
        summary_f.write("="*70 + "\n")
        summary_f.write("CONSOLIDATED POINT CLOUD ANALYSIS SUMMARY\n")
        summary_f.write("Max Amplitude as Center Method\n")
        summary_f.write(f"{dB}dB Threshold\n")
        summary_f.write("="*70 + "\n\n")
        
        # Process each defect
        for defect in defects:
            input_file = f'{base_path}{defect}.ply'
            
            print("\n" + "="*70)
            print(f"Processing {defect}")
            print("="*70)
            
            try:
                # Read point cloud
                print("Reading point cloud...")
                points = read_ply(input_file)
                print(f"Loaded {len(points)} points")
                
                # Process point cloud
                print("\nProcessing point cloud...")
                filtered_points, center, diameter, max_amplitude_center, threshold_amplitude = process_point_cloud(points, dB)
                
                # Save individual filtered point cloud
                output_file = f'{base_path}{defect}_{dB}dB_filtered_point_cloud_maxcenter.txt'
                np.savetxt(output_file, filtered_points, 
                           header='x y z amplitude', 
                           fmt='%.6f',
                           comments='')
                print(f"\nFiltered point cloud saved to: {output_file}")
                
                # Append to consolidated summary
                summary_f.write(f"\n{defect}\n")
                summary_f.write("-" * 70 + "\n")
                summary_f.write(f"Center coordinates: [{center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f}]\n")
                summary_f.write(f"Approximate diameter: {diameter:.6f}\n")
                summary_f.write(f"Original points: {len(points)}\n")
                summary_f.write(f"Filtered points: {len(filtered_points)}\n")
                summary_f.write(f"Points removed: {len(points) - len(filtered_points)}\n")
                summary_f.write(f"Percentage kept: {100*len(filtered_points)/len(points):.1f}%\n")
                summary_f.write("\n")
                
                # Create visualization
                print("\nCreating visualization...")
                create_visualization(points, filtered_points, center, diameter, max_amplitude_center, threshold_amplitude, defect, dB)
                
                print(f"\n✓ Successfully processed {defect}")
                
            except FileNotFoundError:
                print(f"\n✗ Error: File not found - {input_file}")
                summary_f.write(f"\n{defect}\n")
                summary_f.write("-" * 70 + "\n")
                summary_f.write(f"ERROR: File not found\n\n")
                continue
            except Exception as e:
                print(f"\n✗ Error processing {defect}: {str(e)}")
                summary_f.write(f"\n{defect}\n")
                summary_f.write("-" * 70 + "\n")
                summary_f.write(f"ERROR: {str(e)}\n\n")
                continue
        
        summary_f.write("="*70 + "\n")
        summary_f.write("END OF ANALYSIS\n")
        summary_f.write("="*70 + "\n")
    
    print("\n" + "="*70)
    print("Processing complete!")
    print(f"Consolidated summary saved to: {consolidated_summary_file}")
    print("="*70)

def create_visualization(original_points, filtered_points, center, diameter, max_center_amp, threshold, defect, dB):
    """Create comprehensive visualization of the point cloud analysis."""
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    # Fit plane to original point cloud
    normal, plane_basis_1, plane_basis_2, centroid = fit_plane_pca(original_points)
    
    # Project points to plane coordinates
    u_orig, v_orig, w_orig = project_to_plane_coords(original_points, plane_basis_1, plane_basis_2, normal, centroid)
    u_filt, v_filt, w_filt = project_to_plane_coords(filtered_points, plane_basis_1, plane_basis_2, normal, centroid)
    
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
    ax2.set_title(f'Filtered Point Cloud\n(Above {dB}dB Threshold from Max)')
    plt.colorbar(scatter2, ax=ax2, label='Amplitude', shrink=0.5)
    
    # 3. Amplitude histogram
    ax3 = fig.add_subplot(2, 4, 3)
    ax3.hist(amp_orig, bins=50, edgecolor='black', alpha=0.7, label='Original')
    ax3.axvline(max_center_amp, color='r', linestyle='--', linewidth=2, label=f'Max amplitude: {max_center_amp:.3f}')
    ax3.axvline(threshold, color='orange', linestyle='--', linewidth=2, label=f'{dB}dB threshold: {threshold:.3f}')
    ax3.set_xlabel('Normalized Amplitude')
    ax3.set_ylabel('Count')
    ax3.set_title('Amplitude Distribution')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # 4. Statistics panel
    ax4 = fig.add_subplot(2, 4, 4)
    ax4.axis('off')
    stats_text = f"""Analysis Statistics (Max as Center):
━━━━━━━━━━━━━━━━━━━━━━━━━━
Amplitude (Normalized 0-1):
  Min:     {np.min(amp_orig):.6f}
  Max:     {np.max(amp_orig):.6f}
  Mean:    {np.mean(amp_orig):.6f}
  Median:  {np.median(amp_orig):.6f}

Max Amplitude:  {max_center_amp:.6f}
{dB}dB Threshold: {threshold:.6f}

Point Counts:
  Original:  {len(original_points)}
  Filtered:  {len(filtered_points)}
  Removed:   {len(original_points) - len(filtered_points)}
  (Kept {100*len(filtered_points)/len(original_points):.1f}%)

Filtered Cloud:
  Center: [{center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f}]
  Diameter: {diameter:.6f}

Plane Fit:
  Normal: [{normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f}]
"""
    ax4.text(0.05, 0.5, stats_text, fontsize=9, family='monospace',
             verticalalignment='center', transform=ax4.transAxes)
    
    # 5. In-plane view (U-V) - Original
    ax5 = fig.add_subplot(2, 4, 5)
    scatter5 = ax5.scatter(u_orig, v_orig, c=amp_orig, cmap='viridis', s=30, alpha=0.6)
    ax5.set_xlabel('U (Primary plane direction)')
    ax5.set_ylabel('V (Secondary plane direction)')
    ax5.set_title('In-Plane View - Original')
    ax5.axis('equal')
    plt.colorbar(scatter5, ax=ax5, label='Amplitude', shrink=0.8)
    ax5.grid(True, alpha=0.3)
    
    # 6. In-plane view (U-V) - Filtered
    ax6 = fig.add_subplot(2, 4, 6)
    scatter6 = ax6.scatter(u_filt, v_filt, c=amp_filt, cmap='viridis', s=30, alpha=0.6)
    ax6.set_xlabel('U (Primary plane direction)')
    ax6.set_ylabel('V (Secondary plane direction)')
    ax6.set_title('In-Plane View - Filtered')
    ax6.axis('equal')
    plt.colorbar(scatter6, ax=ax6, label='Amplitude', shrink=0.8)
    ax6.grid(True, alpha=0.3)
    
    # 7. Perpendicular view (U-W) - Original
    ax7 = fig.add_subplot(2, 4, 7)
    scatter7 = ax7.scatter(u_orig, w_orig, c=amp_orig, cmap='viridis', s=30, alpha=0.6)
    ax7.set_xlabel('U (Primary plane direction)')
    ax7.set_ylabel('W (Perpendicular to plane)')
    ax7.set_title('Perpendicular View 1 - Original')
    ax7.axis('equal')
    plt.colorbar(scatter7, ax=ax7, label='Amplitude', shrink=0.8)
    ax7.grid(True, alpha=0.3)
    
    # 8. Perpendicular view (U-W) - Filtered
    ax8 = fig.add_subplot(2, 4, 8)
    scatter8 = ax8.scatter(u_filt, w_filt, c=amp_filt, cmap='viridis', s=30, alpha=0.6)
    ax8.set_xlabel('U (Primary plane direction)')
    ax8.set_ylabel('W (Perpendicular to plane)')
    ax8.set_title('Perpendicular View 1 - Filtered')
    ax8.axis('equal')
    plt.colorbar(scatter8, ax=ax8, label='Amplitude', shrink=0.8)
    ax8.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save the figure
    output_path = f'/home/rebecca/ut_servoing/src/peak_ros/src/peak_ros/bags/processed_data_220Gain/Defects_2/{defect}_{dB}dB_analysis_visualization_maxcenter.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Visualization saved to: {output_path}")
    plt.close()

if __name__ == "__main__":
    main()