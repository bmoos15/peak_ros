import numpy as np
from plyfile import PlyData, PlyElement
import sys

def read_ply(filename):
    """Read PLY file and extract x, y, z, and amplitude data."""
    plydata = PlyData.read(filename)
    vertex = plydata['vertex']
    
    x = vertex['x']
    y = vertex['y']
    z = vertex['z']
    
    amplitude_field = None
    for prop in vertex.properties:
        prop_name = prop.name.lower()
        if 'amplitude' in prop_name or 'intensity' in prop_name or 'scalar' in prop_name:
            amplitude_field = prop.name
            break
    
    if amplitude_field is None:
        print("Available properties:", [prop.name for prop in vertex.properties])
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


def save_ply(filepath, points, defect_id=None):
    """
    Save an Nx4 array (x, y, z, amplitude) to a PLY file.
    Optionally includes a uint8 defect_id property per point.

    Args:
        filepath:  Output .ply path
        points:    Nx4 float array (x, y, z, amplitude)
        defect_id: Optional integer label written as a uint8 'defect_id' field
    """
    if defect_id is not None:
        dtype = [
            ('x',         'f4'),
            ('y',         'f4'),
            ('z',         'f4'),
            ('amplitude', 'f4'),
            ('defect_id', 'u1'),
        ]
        records = np.array(
            [(p[0], p[1], p[2], p[3], int(defect_id)) for p in points],
            dtype=dtype,
        )
    else:
        dtype = [
            ('x',         'f4'),
            ('y',         'f4'),
            ('z',         'f4'),
            ('amplitude', 'f4'),
        ]
        records = np.array(
            [(p[0], p[1], p[2], p[3]) for p in points],
            dtype=dtype,
        )

    el = PlyElement.describe(records, 'vertex')
    PlyData([el], text=False).write(filepath)
    print(f"PLY saved → {filepath}  ({len(points)} points)")


def fit_plane_pca(points):
    """
    Fit a plane to the point cloud using PCA.
    Returns the plane normal and two orthogonal basis vectors in the plane.
    """
    xyz = points[:, :3]
    centroid = np.mean(xyz, axis=0)
    centered = xyz - centroid
    
    covariance = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eig(covariance)
    
    idx = eigenvalues.argsort()[::-1]
    eigenvalues = eigenvalues[idx]
    eigenvectors = eigenvectors[:, idx]
    
    normal = eigenvectors[:, 2]
    plane_basis_1 = eigenvectors[:, 0]
    plane_basis_2 = eigenvectors[:, 1]
    
    print(f"\nPlane fitting (PCA):")
    print(f"Centroid: {centroid}")
    print(f"Normal vector: {normal}")
    print(f"Eigenvalues: {eigenvalues}")
    print(f"Plane basis 1: {plane_basis_1}")
    print(f"Plane basis 2: {plane_basis_2}")
    
    return normal, plane_basis_1, plane_basis_2, centroid

def find_peak_center_and_amplitude(points, n_top=5):
    """
    Find the center and reference amplitude using the top n_top
    highest-amplitude points.

    Returns:
        center:           Mean XYZ of the top n_top points (3-element array)
        center_amplitude: Mean amplitude of the top n_top points
    """
    top_idx = np.argsort(points[:, 3])[-n_top:]
    top_points = points[top_idx]

    center = np.mean(top_points[:, :3], axis=0)
    center_amplitude = np.mean(top_points[:, 3])

    print(f"\nPeak center (top {n_top} amplitude points):")
    print(f"  Center:           [{center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f}]")
    print(f"  Center amplitude: {center_amplitude:.6f}")

    return center, center_amplitude

def project_to_plane_coords(points, plane_basis_1, plane_basis_2, normal, centroid):
    """
    Project points to 2D coordinates using the plane basis vectors.
    Returns coordinates in the plane and perpendicular to the plane.
    """
    xyz = points[:, :3]
    centered = xyz - centroid
    
    u = np.dot(centered, plane_basis_1)
    v = np.dot(centered, plane_basis_2)
    w = np.dot(centered, normal)
    
    return u, v, w

# def interpolate_amplitude_at_center(points, center, n_neighbours=5):
#     """
#     Estimate amplitude at the geometric center using inverse-distance
#     weighted interpolation from the nearest neighbours.

#     Args:
#         points:       Nx4 array (x, y, z, amplitude)
#         center:       3-element array, geometric center coordinates
#         n_neighbours: how many nearest points to use

#     Returns:
#         Interpolated amplitude at the center
#     """
#     distances = np.linalg.norm(points[:, :3] - center, axis=1)
#     nearest_idx = np.argsort(distances)[:n_neighbours]
#     nearest_distances = distances[nearest_idx]
#     nearest_amplitudes = points[nearest_idx, 3]

#     # Avoid division by zero if center coincides exactly with a point
#     if nearest_distances[0] < 1e-10:
#         return nearest_amplitudes[0]

#     weights = 1.0 / nearest_distances
#     center_amplitude = np.sum(weights * nearest_amplitudes) / np.sum(weights)
#     return center_amplitude

def find_radial_edge_points(u, v, amplitudes, center_amplitude, dB, n_angles=360):
    """
    Scan outward from the origin (geometric center in plane coordinates)
    in n_angles directions. In each direction, find the first point whose
    amplitude drops dB below center_amplitude. Return the edge points.

    Strategy per radial line:
      - Collect all points within a narrow angular bin around each direction.
      - Sort them by distance from the origin.
      - Walk outward; the edge is where amplitude first crosses the threshold.
      - If no crossing is found (all points are above threshold), use the
        furthest point in that bin as the edge.
      - If the bin is empty, skip that direction.

    Args:
        u, v:              In-plane coordinates (centered on geometric center)
        amplitudes:        Amplitude values for each point
        center_amplitude:  Amplitude at the geometric center
        dB:                dB drop that defines the edge
        n_angles:          Number of radial directions to sample

    Returns:
        edge_u, edge_v:    In-plane coordinates of edge points
        edge_radii:        Distance from center for each edge point
        edge_angles:       Angle (radians) for each edge point
    """
    threshold = center_amplitude * 10**(-dB / 20)
    print(f"\nRadial edge detection:")
    print(f"Center amplitude: {center_amplitude:.6f}")
    print(f"Edge threshold ({dB}dB drop): {threshold:.6f}")

    angles = np.linspace(0, 2 * np.pi, n_angles, endpoint=False)
    half_bin = np.pi / n_angles  # Half the angular bin width

    # Pre-compute polar coords for all points
    point_angles = np.arctan2(v, u)          # [-pi, pi]
    point_radii  = np.sqrt(u**2 + v**2)

    edge_u, edge_v, edge_radii, edge_angles = [], [], [], []

    for angle in angles:
        # Angular difference, wrapped to [-pi, pi]
        delta = point_angles - angle
        delta = (delta + np.pi) % (2 * np.pi) - np.pi
        in_bin = np.abs(delta) <= half_bin

        if not np.any(in_bin):
            continue

        bin_radii     = point_radii[in_bin]
        bin_amplitudes = amplitudes[in_bin]

        # Sort by distance from center (nearest first)
        order = np.argsort(bin_radii)
        bin_radii      = bin_radii[order]
        bin_amplitudes = bin_amplitudes[order]

        # Walk outward to find first crossing below threshold
        edge_radius = bin_radii[-1]  # Default: furthest point in bin
        for r, amp in zip(bin_radii, bin_amplitudes):
            if amp < threshold:
                edge_radius = r
                break

        edge_u.append(edge_radius * np.cos(angle))
        edge_v.append(edge_radius * np.sin(angle))
        edge_radii.append(edge_radius)
        edge_angles.append(angle)

    edge_u      = np.array(edge_u)
    edge_v      = np.array(edge_v)
    edge_radii  = np.array(edge_radii)
    edge_angles = np.array(edge_angles)

    print(f"Edge found in {len(edge_radii)} / {n_angles} directions")
    print(f"Mean edge radius: {np.mean(edge_radii):.6f}")
    print(f"Min edge radius:  {np.min(edge_radii):.6f}")
    print(f"Max edge radius:  {np.max(edge_radii):.6f}")
    print(f"Diameter (mean):  {2 * np.mean(edge_radii):.6f}")

    return edge_u, edge_v, edge_radii, edge_angles

def process_point_cloud(points, dB):
    """
    Process point cloud:
      1. Normalise amplitudes.
      2. Find center and reference amplitude as the mean of the top 50
         highest-amplitude points.
      3. Apply dB threshold from that center amplitude.
      4. Fit plane to points above threshold and project to 2D.
      5. Scan radially to find the dB edge in each direction.

    Args:
        points: Nx4 array with x, y, z, amplitude columns
        dB:     dB drop that defines the edge

    Returns:
        filtered_points:   Points inside the dB edge (above threshold)
        center:            Peak center (3D, mean of top-50 points)
        center_amplitude:  Reference amplitude (mean of top-50 amplitudes)
        threshold:         Amplitude threshold used
        edge info, plane info
    """
    # --- Normalise ---
    min_amp = np.min(points[:, 3])
    max_amp = np.max(points[:, 3])
    print(f"Original amplitude range: [{min_amp:.6f}, {max_amp:.6f}]")
    points[:, 3] = (points[:, 3] - min_amp) / (max_amp - min_amp)
    print(f"Normalized amplitude range: [{np.min(points[:, 3]):.6f}, {np.max(points[:, 3]):.6f}]")

    # --- Center and amplitude from top-50 highest-amplitude points ---
    center, center_amplitude = find_peak_center_and_amplitude(points, n_top=50)
    print(f"Reference amplitude (mean of top 50): {center_amplitude:.6f}")

    # --- Threshold: dB drop below center amplitude ---
    threshold = center_amplitude * 10**(-dB / 20)
    print(f"Edge threshold ({dB}dB below center): {threshold:.6f}")

    # --- Filter: keep only points above threshold ---
    filtered_points = points[points[:, 3] >= threshold]
    print(f"\nOriginal point count:     {len(points)}")
    print(f"Points above threshold:   {len(filtered_points)}")

    # --- Fit plane to filtered points ---
    normal, plane_basis_1, plane_basis_2, centroid = fit_plane_pca(filtered_points)

    # --- Project ALL points to plane coords centred on peak center ---
    centered_all = points[:, :3] - center
    u_all = np.dot(centered_all, plane_basis_1)
    v_all = np.dot(centered_all, plane_basis_2)

    # --- Radial edge detection ---
    edge_u, edge_v, edge_radii, edge_angles = find_radial_edge_points(
        u_all, v_all, points[:, 3], center_amplitude, dB, n_angles=360
    )

    # --- Diameter: mean of radial edge distances ---
    centered_filt = filtered_points[:, :3] - center
    u_filt = np.dot(centered_filt, plane_basis_1)
    v_filt = np.dot(centered_filt, plane_basis_2)
    filt_radii = np.sqrt(u_filt**2 + v_filt**2)
    diameter = 2 * np.mean(filt_radii)
    print(f"\nFiltered-point diameter estimate:")
    print(f"  Mean in-plane radius : {np.mean(filt_radii):.6f}")
    print(f"  Diameter (2 × mean)  : {diameter:.6f}")

    return (filtered_points, center, center_amplitude, threshold,
            edge_u, edge_v, edge_radii, edge_angles,
            normal, plane_basis_1, plane_basis_2, centroid, diameter)

def main():
    dB = 3
    Gain = 173
    TVG = 4

    # defects = ['Defect_1_2TVG', 'Defect_2_2TVG', 'Defect_3_2TVG', 'Defect_4_2TVG', 'Defect_5_2TVG',
    #            'Defect_6_2TVG', 'Defect_7_2TVG', 'Defect_8_2TVG', 'Defect_9_2TVG', 'Defect_10_2TVG']

    # defects = ['Defect_1__8TVG', 'Defect_2__8TVG', 'Defect_3__8TVG', 'Defect_4__8TVG', 'Defect_5__8TVG',
               # 'Defect_6__8TVG', 'Defect_7__8TVG', 'Defect_8__8TVG', 'Defect_9__8TVG', 'Defect_10__8TVG']

    # defects = ['Defect_01_160Gain_2TVG', 'Defect_02_160Gain_2TVG', 'Defect_03_160Gain_2TVG', 'Defect_04_160Gain_2TVG',
    #             'Defect_05_160Gain_2TVG', 'Defect_06_160Gain_2TVG', 'Defect_07_160Gain_2TVG', 'Defect_08_160Gain_2TVG',
    #             'Defect_09_160Gain_2TVG', 'Defect_10_160Gain_2TVG']

    defects = [f'Defect_01_{Gain}Gain_{TVG}TVG', f'Defect_02_{Gain}Gain_{TVG}TVG', f'Defect_03_{Gain}Gain_{TVG}TVG', f'Defect_04_{Gain}Gain_{TVG}TVG',
            f'Defect_05_{Gain}Gain_{TVG}TVG', f'Defect_06_{Gain}Gain_{TVG}TVG', f'Defect_07_{Gain}Gain_{TVG}TVG', f'Defect_08_{Gain}Gain_{TVG}TVG',
            f'Defect_09_{Gain}Gain_{TVG}TVG', f'Defect_10_{Gain}Gain_{TVG}TVG',]

    base_path = '/home/rebecca/ut_servoing/src/peak_ros/src/peak_ros/bags/New_angle/Corr/Gate_5/'

    consolidated_summary_file = f'{base_path}{dB}dB_radial_analysis_summary.txt'

    # Accumulate all filtered points for the combined PLY
    all_filtered_points = []   # list of Nx4 arrays
    all_defect_ids      = []   # matching defect index per point

    with open(consolidated_summary_file, 'w') as summary_f:
        summary_f.write("="*70 + "\n")
        summary_f.write("CONSOLIDATED POINT CLOUD ANALYSIS SUMMARY\n")
        summary_f.write("Radial Edge Detection Method\n")
        summary_f.write(f"{dB}dB Drop from Geometric Center Amplitude\n")
        summary_f.write("="*70 + "\n\n")

        for defect_idx, defect in enumerate(defects):
            input_file = f'{base_path}{defect}.ply'

            print("\n" + "="*70)
            print(f"Processing {defect}")
            print("="*70)

            try:
                print("Reading point cloud...")
                points = read_ply(input_file)
                print(f"Loaded {len(points)} points")

                print("\nProcessing point cloud...")
                (filtered_points, center, center_amplitude, threshold,
                 edge_u, edge_v, edge_radii, edge_angles,
                 normal, plane_basis_1, plane_basis_2, centroid,
                 diameter) = process_point_cloud(points, dB)

                # ── Per-defect PLY ──────────────────────────────────────────
                ply_output = f'{base_path}{defect}_{dB}dB_filtered_radial.ply'
                save_ply(ply_output, filtered_points, defect_id=defect_idx)

                # Accumulate for combined file
                all_filtered_points.append(filtered_points)
                all_defect_ids.append(
                    np.full(len(filtered_points), defect_idx, dtype=np.uint8)
                )

                # Save edge points (in plane coords) — kept as txt
                edge_output = f'{base_path}{defect}_{dB}dB_edge_points.txt'
                edge_data = np.column_stack([edge_u, edge_v, edge_radii, np.degrees(edge_angles)])
                np.savetxt(edge_output, edge_data,
                           header='u v radius angle_deg',
                           fmt='%.6f',
                           comments='')
                print(f"Edge points saved to: {edge_output}")

                summary_f.write(f"\n{defect}\n")
                summary_f.write("-" * 70 + "\n")
                summary_f.write(f"Geometric center: [{center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f}]\n")
                summary_f.write(f"Center amplitude (interpolated): {center_amplitude:.6f}\n")
                summary_f.write(f"Edge threshold ({dB}dB drop): {threshold:.6f}\n")
                summary_f.write(f"Diameter (mean of radial edges): {diameter:.6f}\n")
                summary_f.write(f"Min edge radius: {np.min(edge_radii):.6f}\n")
                summary_f.write(f"Max edge radius: {np.max(edge_radii):.6f}\n")
                summary_f.write(f"Original points: {len(points)}\n")
                summary_f.write(f"Points above threshold: {len(filtered_points)}\n")
                summary_f.write("\n")

                print("\nCreating visualization...")
                create_visualization(points, filtered_points, center, center_amplitude,
                                     threshold, edge_u, edge_v, edge_radii,
                                     normal, plane_basis_1, plane_basis_2, centroid,
                                     diameter, defect, dB)

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

    # ── Combined PLY with all defects ──────────────────────────────────────
    if all_filtered_points:
        combined_points = np.vstack(all_filtered_points)
        combined_ids    = np.concatenate(all_defect_ids)

        dtype = [
            ('x',         'f4'),
            ('y',         'f4'),
            ('z',         'f4'),
            ('amplitude', 'f4'),
            ('defect_id', 'u1'),
        ]
        records = np.array(
            [(p[0], p[1], p[2], p[3], int(did))
             for p, did in zip(combined_points, combined_ids)],
            dtype=dtype,
        )
        el = PlyElement.describe(records, 'vertex')
        combined_ply_path = f'{base_path}{dB}dB_all_defects_combined.ply'
        PlyData([el], text=False).write(combined_ply_path)
        print(f"\nCombined PLY saved → {combined_ply_path}  ({len(combined_points)} total points)")

    print("\n" + "="*70)
    print("Processing complete!")
    print(f"Consolidated summary saved to: {consolidated_summary_file}")
    print("="*70)

def create_visualization(original_points, filtered_points, center, center_amplitude,
                         threshold, edge_u, edge_v, edge_radii,
                         normal, plane_basis_1, plane_basis_2, centroid,
                         diameter, defect, dB):
    """Create comprehensive visualization of the point cloud analysis."""
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    # Project points to plane coords (centered on geometric center)
    xyz_all = original_points[:, :3]
    centered_all = xyz_all - center
    u_orig = np.dot(centered_all, plane_basis_1)
    v_orig = np.dot(centered_all, plane_basis_2)
    w_orig = np.dot(centered_all, normal)
    amp_orig = original_points[:, 3]

    xyz_filt = filtered_points[:, :3]
    centered_filt = xyz_filt - center
    u_filt = np.dot(centered_filt, plane_basis_1)
    v_filt = np.dot(centered_filt, plane_basis_2)
    w_filt = np.dot(centered_filt, normal)
    amp_filt = filtered_points[:, 3]

    # Close the edge polygon for plotting
    edge_u_closed = np.append(edge_u, edge_u[0])
    edge_v_closed = np.append(edge_v, edge_v[0])

    fig = plt.figure(figsize=(18, 12))

    # 1. Original 3D scatter
    ax1 = fig.add_subplot(2, 4, 1, projection='3d')
    scatter1 = ax1.scatter(original_points[:, 0], original_points[:, 1], original_points[:, 2],
                           c=amp_orig, cmap='viridis', s=20, alpha=0.6)
    ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')
    ax1.set_title('Original Point Cloud\n(Normalized Amplitude)')
    plt.colorbar(scatter1, ax=ax1, label='Amplitude', shrink=0.5)

    # 2. Filtered 3D scatter with center marked
    ax2 = fig.add_subplot(2, 4, 2, projection='3d')
    scatter2 = ax2.scatter(filtered_points[:, 0], filtered_points[:, 1], filtered_points[:, 2],
                           c=amp_filt, cmap='viridis', s=20, alpha=0.6)
    ax2.scatter(*center, color='red', s=100, marker='*', label='Geometric center', zorder=5)
    ax2.set_xlabel('X'); ax2.set_ylabel('Y'); ax2.set_zlabel('Z')
    ax2.set_title(f'Filtered Point Cloud\n(Above {dB}dB Threshold from Center)')
    ax2.legend(fontsize=7)
    plt.colorbar(scatter2, ax=ax2, label='Amplitude', shrink=0.5)

    # 3. Amplitude histogram
    ax3 = fig.add_subplot(2, 4, 3)
    ax3.hist(amp_orig, bins=50, edgecolor='black', alpha=0.7, label='Original')
    ax3.axvline(center_amplitude, color='r', linestyle='--', linewidth=2,
                label=f'Center amplitude: {center_amplitude:.3f}')
    ax3.axvline(threshold, color='orange', linestyle='--', linewidth=2,
                label=f'{dB}dB threshold: {threshold:.3f}')
    ax3.set_xlabel('Normalized Amplitude')
    ax3.set_ylabel('Count')
    ax3.set_title('Amplitude Distribution')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # 4. Statistics panel
    ax4 = fig.add_subplot(2, 4, 4)
    ax4.axis('off')
    stats_text = f"""Analysis Statistics (Radial Edge):
━━━━━━━━━━━━━━━━━━━━━━━━━━
Amplitude (Normalized 0-1):
  Min:     {np.min(amp_orig):.6f}
  Max:     {np.max(amp_orig):.6f}
  Mean:    {np.mean(amp_orig):.6f}
  Median:  {np.median(amp_orig):.6f}

Center amplitude: {center_amplitude:.6f}
{dB}dB threshold: {threshold:.6f}

Point Counts:
  Original:  {len(original_points)}
  Filtered:  {len(filtered_points)}
  Removed:   {len(original_points) - len(filtered_points)}
  (Kept {100*len(filtered_points)/len(original_points):.1f}%)

Radial Edge:
  Mean radius:  {np.mean(edge_radii):.6f}
  Min radius:   {np.min(edge_radii):.6f}
  Max radius:   {np.max(edge_radii):.6f}
  Diameter:     {diameter:.6f}

Geometric center:
  [{center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f}]

Plane normal:
  [{normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f}]
"""
    ax4.text(0.05, 0.5, stats_text, fontsize=9, family='monospace',
             verticalalignment='center', transform=ax4.transAxes)

    # 5. In-plane view - Original
    ax5 = fig.add_subplot(2, 4, 5)
    scatter5 = ax5.scatter(u_orig, v_orig, c=amp_orig, cmap='viridis', s=30, alpha=0.6)
    ax5.set_xlabel('U (Primary plane direction)')
    ax5.set_ylabel('V (Secondary plane direction)')
    ax5.set_title('In-Plane View - Original')
    ax5.axis('equal')
    plt.colorbar(scatter5, ax=ax5, label='Amplitude', shrink=0.8)
    ax5.grid(True, alpha=0.3)

    # 6. In-plane view - Filtered with radial edge and center
    ax6 = fig.add_subplot(2, 4, 6)
    scatter6 = ax6.scatter(u_filt, v_filt, c=amp_filt, cmap='viridis', s=30, alpha=0.6)
    ax6.plot(edge_u_closed, edge_v_closed, 'r-', linewidth=2, label=f'{dB}dB edge')
    ax6.scatter(0, 0, color='red', s=150, marker='*', label='Geometric center', zorder=5)
    ax6.set_xlabel('U (Primary plane direction)')
    ax6.set_ylabel('V (Secondary plane direction)')
    ax6.set_title(f'In-Plane View - Filtered\nwith {dB}dB Radial Edge')
    ax6.axis('equal')
    ax6.legend(fontsize=7)
    plt.colorbar(scatter6, ax=ax6, label='Amplitude', shrink=0.8)
    ax6.grid(True, alpha=0.3)

    # 7. Radial profile — amplitude vs distance from center
    ax7 = fig.add_subplot(2, 4, 7)
    point_radii_all = np.sqrt(u_orig**2 + v_orig**2)
    ax7.scatter(point_radii_all, amp_orig, s=10, alpha=0.4, label='All points')
    ax7.axhline(center_amplitude, color='r', linestyle='--', linewidth=2,
                label=f'Center amplitude: {center_amplitude:.3f}')
    ax7.axhline(threshold, color='orange', linestyle='--', linewidth=2,
                label=f'{dB}dB threshold: {threshold:.3f}')
    ax7.set_xlabel('Distance from geometric center')
    ax7.set_ylabel('Normalized Amplitude')
    ax7.set_title('Radial Amplitude Profile')
    ax7.legend(fontsize=7)
    ax7.grid(True, alpha=0.3)

    # 8. Edge radius vs angle (polar-style in cartesian)
    ax8 = fig.add_subplot(2, 4, 8, projection='polar')
    ax8.plot(edge_angles, edge_radii, 'r-', linewidth=2)
    ax8.fill(edge_angles, edge_radii, alpha=0.2, color='red')
    ax8.set_title(f'{dB}dB Edge Profile\n(Polar)', pad=15)

    plt.suptitle(f'{defect} — {dB}dB Radial Edge Detection', fontsize=13, fontweight='bold')
    plt.tight_layout()

    output_path = f'/home/rebecca/ut_servoing/src/peak_ros/src/peak_ros/bags/New_angle/Corr/{defect}_{dB}dB_radial_summary.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Visualization saved to: {output_path}")
    plt.close()

if __name__ == "__main__":
    main()