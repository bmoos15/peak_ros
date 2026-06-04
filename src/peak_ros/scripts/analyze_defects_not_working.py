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
    
    # print(f"\nPlane fitting (PCA):")
    # print(f"Centroid: {centroid}")
    # print(f"Normal vector: {normal}")
    # print(f"Eigenvalues: {eigenvalues}")
    # print(f"Plane basis 1: {plane_basis_1}")
    # print(f"Plane basis 2: {plane_basis_2}")
    
    return normal, plane_basis_1, plane_basis_2, centroid


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


def find_radial_edge_points(u, v, amplitudes, center_amplitude, dB, n_confirm = 3, n_angles=360):
    """
    Scan outward from the origin in n_angles directions. In each direction,
    find the first point whose amplitude drops dB below center_amplitude,
    confirmed by requiring that point AND its n_confirm nearest neighbours
    (by radius within the bin) also fall below threshold.

    Args:
        u, v:              In-plane coordinates (centered on geometric center)
        amplitudes:        Amplitude values for each point
        center_amplitude:  Amplitude at the geometric center
        dB:                dB drop that defines the edge
        n_angles:          Number of radial directions to sample
        n_confirm:         Number of neighbours that must also cross threshold

    Returns:
        edge_u, edge_v:    In-plane coordinates of edge points
        edge_radii:        Distance from center for each edge point
        edge_angles:       Angle (radians) for each edge point
    """
    threshold = center_amplitude * 10**(-dB / 20)
    print(f"\nRadial edge detection:")
    print(f"Center amplitude: {center_amplitude:.6f}")
    print(f"Edge threshold ({dB}dB drop): {threshold:.6f}")
    print(f"Confirmation neighbours required: {n_confirm}")

    angles = np.linspace(0, 2 * np.pi, n_angles, endpoint=False)
    half_bin = np.pi / n_angles

    point_angles = np.arctan2(v, u)
    point_radii  = np.sqrt(u**2 + v**2)

    edge_u, edge_v, edge_radii, edge_angles = [], [], [], []

    for angle in angles:
        delta = point_angles - angle
        delta = (delta + np.pi) % (2 * np.pi) - np.pi
        in_bin = np.abs(delta) <= half_bin

        if not np.any(in_bin):
            continue

        bin_radii      = point_radii[in_bin]
        bin_amplitudes = amplitudes[in_bin]

        order          = np.argsort(bin_radii)
        bin_radii      = bin_radii[order]
        bin_amplitudes = bin_amplitudes[order]

        edge_radius = bin_radii[-1]  # Default: furthest point if no confirmed crossing

        for i, (r, amp) in enumerate(zip(bin_radii, bin_amplitudes)):
            if amp < threshold:
                # Gather the n_confirm nearest neighbours by radius (excluding point i)
                distances_from_i = np.abs(bin_radii - r)
                distances_from_i[i] = np.inf  # exclude self
                neighbour_idx = np.argsort(distances_from_i)[:n_confirm]

                if len(neighbour_idx) < n_confirm:
                    # Not enough neighbours in this bin — skip, keep searching
                    continue

                if np.all(bin_amplitudes[neighbour_idx] < threshold):
                    edge_radius = r
                    break
                # else: crossing not confirmed, keep walking outward

        edge_u.append(edge_radius * np.cos(angle))
        edge_v.append(edge_radius * np.sin(angle))
        edge_radii.append(edge_radius)
        edge_angles.append(angle)

    edge_u      = np.array(edge_u)
    edge_v      = np.array(edge_v)
    edge_radii  = np.array(edge_radii)
    edge_angles = np.array(edge_angles)

    print(f"Edge found in {len(edge_radii)} / {n_angles} directions")
    # print(f"Mean edge radius: {np.mean(edge_radii):.6f}")
    # print(f"Min edge radius:  {np.min(edge_radii):.6f}")
    # print(f"Max edge radius:  {np.max(edge_radii):.6f}")
    # print(f"Diameter (mean):  {2 * np.mean(edge_radii):.6f}")

    return edge_u, edge_v, edge_radii, edge_angles


def estimate_cardinal_dimensions(edge_radii, edge_angles, tolerance_deg=5.0):
    """
    Estimate horizontal (0°/180°) and vertical (90°/270°) dimensions
    from the radial edge profile.

    For each cardinal direction, finds the edge radius at the closest
    sampled angle within tolerance_deg.

    Args:
        edge_radii:     Array of edge radii
        edge_angles:    Array of edge angles in radians
        tolerance_deg:  Max angular distance to accept a match (degrees)

    Returns:
        dict with keys: r_0, r_90, r_180, r_270,
                        width (0+180), height (90+270)
    """
    tol = np.radians(tolerance_deg)

    def radii_near_angle(target_deg):
        """Return mean of edge radii whose angles are within tol of target."""
        delta = np.abs(edge_angles - target_deg)
        delta = np.minimum(delta, 2 * np.pi - delta)
        nearby = edge_radii[delta <= tol]
        return np.mean(nearby) if len(nearby) > 0 else np.nan

    r_0   = radii_near_angle(0.0)
    r_90  = radii_near_angle(np.pi / 2)
    r_180 = radii_near_angle(np.pi)
    r_270 = radii_near_angle(3 * np.pi / 2)

    width  = np.sum([r_0, r_180])   # nansum so a single nan doesn't kill it
    height = np.sum([r_90, r_270])

    results = {
        'r_0': r_0, 'r_90': r_90, 'r_180': r_180, 'r_270': r_270,
        'width': width, 'height': height
    }

    print(f"\nCardinal Diameter: {((width + height) / 2):.6f}")

    return results


def process_point_cloud(points, dB):
    """
    Process point cloud:
      1. Normalise amplitudes.
      2. Find center as geometric mean; estimate amplitude at center via IDW.
      3. Apply dB threshold from that center amplitude.
      4. Fit plane to points above threshold and project to 2D.
      5. Scan radially (with neighbour confirmation) to find the dB edge.
      6. Estimate cardinal (horizontal/vertical) dimensions from edge profile.
    """
    # --- Normalise ---
    min_amp = np.min(points[:, 3])
    max_amp = np.max(points[:, 3])
    print(f"Original amplitude range: [{min_amp:.6f}, {max_amp:.6f}]")
    points[:, 3] = (points[:, 3] - min_amp) / (max_amp - min_amp)
    print(f"Normalized amplitude range: [{np.min(points[:, 3]):.6f}, {np.max(points[:, 3]):.6f}]")

    # --- Geometric centre and IDW amplitude ---
    center = np.mean(points[:, :3], axis=0)
    print(f"\nGeometric centre: [{center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f}]")

    # distances = np.linalg.norm(points[:, :3] - center, axis=1)
    # valid_mask = ~np.isnan(points[:, 3])
    # valid_distances = distances[valid_mask]
    # valid_amplitudes = points[valid_mask, 3]

    # nearest_idx = np.argsort(valid_distances)[:20]
    # nearest_amplitudes = valid_amplitudes[nearest_idx]
    # nearest_distances  = valid_distances[nearest_idx]

    # weights = 1.0 / (nearest_distances + 1e-10)
    center_amplitude = np.max(points[:, 4])
    print(f"Amplitude at geometric centre (IDW, top 20 non-NaN): {center_amplitude:.6f}")

    # --- Threshold ---
    threshold = center_amplitude * 10**(-dB / 20)
    # print(f"Edge threshold ({dB}dB below center): {threshold:.6f}")

    # --- Filter ---
    filtered_points = points[points[:, 3] >= 0]
    # print(f"\nOriginal point count:     {len(points)}")
    # print(f"Points above threshold:   {len(filtered_points)}")

    # --- Fit plane ---
    normal, plane_basis_1, plane_basis_2, centroid = fit_plane_pca(filtered_points)

    # --- Project ALL points to plane coords centred on geometric centre ---
    centered_all = points[:, :3] - center
    u_all = np.dot(centered_all, plane_basis_1)
    v_all = np.dot(centered_all, plane_basis_2)

    # --- Radial edge detection with neighbour confirmation ---
    edge_u, edge_v, edge_radii, edge_angles = find_radial_edge_points(
        u_all, v_all, points[:, 3], center_amplitude, dB, n_confirm = 3, n_angles=360
    )

    if len(edge_radii) == 0:
        raise ValueError("No edge points found — check point cloud coverage and threshold.")

    # --- Cardinal dimension estimates ---
    cardinal_dims = estimate_cardinal_dimensions(edge_radii, edge_angles)

    # --- Filtered-point radius stats for summary ---
    centered_filt = filtered_points[:, :3] - center
    u_filt = np.dot(centered_filt, plane_basis_1)
    v_filt = np.dot(centered_filt, plane_basis_2)
    filt_radii = np.sqrt(u_filt**2 + v_filt**2)
    diameter = 2 * np.mean(filt_radii)
    # print(f"\nFiltered-point diameter estimate:")
    # print(f"  Mean in-plane radius : {np.mean(filt_radii):.6f}")
    # print(f"  Diameter (2 × mean)  : {diameter:.6f}")

    return (filtered_points, center, center_amplitude, threshold,
            edge_u, edge_v, edge_radii, edge_angles,
            normal, plane_basis_1, plane_basis_2, filt_radii, centroid,
            diameter, cardinal_dims)


def main():
    dB = 6
    Gain = 145
    TVG = 8

    defects = [f'Defect_01_{Gain}Gain_{TVG}TVG', f'Defect_02_{Gain}Gain_{TVG}TVG',
               f'Defect_03_{Gain}Gain_{TVG}TVG', f'Defect_04_{Gain}Gain_{TVG}TVG',
               f'Defect_05_{Gain}Gain_{TVG}TVG', f'Defect_06_{Gain}Gain_{TVG}TVG',
               f'Defect_07_{Gain}Gain_{TVG}TVG', f'Defect_08_{Gain}Gain_{TVG}TVG',
               f'Defect_09_{Gain}Gain_{TVG}TVG', f'Defect_10_{Gain}Gain_{TVG}TVG']

    base_path = '/home/rebecca/ut_servoing/src/peak_ros/src/peak_ros/bags/New_angle/Corr/Hilbert_no_norm/Gain145_8TVG_Merge/'

    consolidated_summary_file = f'{base_path}{dB}dB_radial_analysis_summary.txt'

    all_filtered_points = []
    all_defect_ids      = []

    with open(consolidated_summary_file, 'w') as summary_f:
        summary_f.write("="*70 + "\n")
        summary_f.write("CONSOLIDATED POINT CLOUD ANALYSIS SUMMARY\n")
        summary_f.write("Radial Edge Detection Method (with neighbour confirmation)\n")
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
                 normal, plane_basis_1, plane_basis_2, filt_radii, centroid,
                 diameter, cardinal_dims) = process_point_cloud(points, dB)

                # --- Per-defect PLY ---
                ply_output = f'{base_path}{defect}_{dB}dB_filtered_radial.ply'
                save_ply(ply_output, filtered_points, defect_id=defect_idx)

                all_filtered_points.append(filtered_points)
                all_defect_ids.append(
                    np.full(len(filtered_points), defect_idx, dtype=np.uint8)
                )

                # --- Edge points txt ---
                edge_output = f'{base_path}{defect}_{dB}dB_edge_points.txt'
                edge_data = np.column_stack([edge_u, edge_v, edge_radii, np.degrees(edge_angles)])
                np.savetxt(edge_output, edge_data,
                           header='u v radius angle_deg',
                           fmt='%.6f',
                           comments='')
                print(f"Edge points saved to: {edge_output}")

                # --- Summary ---
                summary_f.write(f"\n{defect}\n")
                summary_f.write("-" * 70 + "\n")
                summary_f.write(f"Geometric center: [{center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f}]\n")
                summary_f.write(f"Center amplitude (IDW): {center_amplitude:.6f}\n")
                summary_f.write(f"Edge threshold ({dB}dB drop): {threshold:.6f}\n")
                summary_f.write(f"Diameter (mean of radial edges): {diameter:.6f}\n")
                # summary_f.write(f"Min edge radius: {np.min(edge_radii):.6f}\n")
                # summary_f.write(f"Max edge radius: {np.max(edge_radii):.6f}\n")
                summary_f.write(f"Horizontal width (r_0° + r_180°): {cardinal_dims['width']:.6f}\n")
                summary_f.write(f"  r_0°={cardinal_dims['r_0']:.6f}  r_180°={cardinal_dims['r_180']:.6f}\n")
                summary_f.write(f"Vertical height (r_90° + r_270°): {cardinal_dims['height']:.6f}\n")
                summary_f.write(f"  r_90°={cardinal_dims['r_90']:.6f}  r_270°={cardinal_dims['r_270']:.6f}\n")
                summary_f.write(f"Average cardinal diameter: {((cardinal_dims['width'] + cardinal_dims['height'])/2):.6f}\n")
                # summary_f.write(f"Original points: {len(points)}\n")
                # summary_f.write(f"Points above threshold: {len(filtered_points)}\n")
                summary_f.write("\n")

                print("\nCreating visualization...")

                create_visualization(points, filtered_points, center, center_amplitude,
                                     threshold, edge_u, edge_v, edge_radii, edge_angles,
                                     normal, plane_basis_1, plane_basis_2, filt_radii, centroid,
                                     diameter, cardinal_dims, defect, dB, base_path)

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

    # --- Combined PLY ---
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
                             threshold, edge_u, edge_v, edge_radii, edge_angles,
                             normal, plane_basis_1, plane_basis_2, filt_radii, centroid,
                             diameter, cardinal_dims, defect, dB, base_path):
    """Create comprehensive visualization of the point cloud analysis."""
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    xyz_all = original_points[:, :3]
    centered_all = xyz_all - center
    u_orig = np.dot(centered_all, plane_basis_1)
    v_orig = np.dot(centered_all, plane_basis_2)
    amp_orig = original_points[:, 3]

    xyz_filt = filtered_points[:, :3]
    centered_filt = xyz_filt - center
    u_filt = np.dot(centered_filt, plane_basis_1)
    v_filt = np.dot(centered_filt, plane_basis_2)
    amp_filt = filtered_points[:, 3]

    # --- Colour masks: red = above threshold, blue = below ---
    above_mask_orig = amp_orig >= threshold
    below_mask_orig = ~above_mask_orig

    above_mask_filt = amp_filt >= threshold
    below_mask_filt = ~above_mask_filt

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

    # 2. Filtered 3D scatter — red above threshold, blue below
    ax2 = fig.add_subplot(2, 4, 2, projection='3d')
    if np.any(above_mask_filt):
        ax2.scatter(filtered_points[above_mask_filt, 0],
                    filtered_points[above_mask_filt, 1],
                    filtered_points[above_mask_filt, 2],
                    c='red', s=20, alpha=0.6, label=f'Above {dB}dB threshold')
    if np.any(below_mask_filt):
        ax2.scatter(filtered_points[below_mask_filt, 0],
                    filtered_points[below_mask_filt, 1],
                    filtered_points[below_mask_filt, 2],
                    c='blue', s=20, alpha=0.6, label=f'Below {dB}dB threshold')
    ax2.scatter(*center, color='yellow', s=100, marker='*', label='Geometric center', zorder=5)
    ax2.set_xlabel('X'); ax2.set_ylabel('Y'); ax2.set_zlabel('Z')
    ax2.set_title(f'Filtered Point Cloud\n(Above {dB}dB Threshold)')
    ax2.legend(fontsize=7)

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
    stats_text = f"""Analysis Statistics:
━━━━━━━━━━━━━━━━━━━━━━━━━━
Center amplitude: {center_amplitude:.6f}
{dB}dB threshold: {threshold:.6f}

Cardinal Diameter: {((cardinal_dims['width'] + cardinal_dims['height'])/2):.6f}

"""
    ax4.text(0.05, 0.5, stats_text, fontsize=9, family='monospace',
             verticalalignment='center', transform=ax4.transAxes)

    # 5. In-plane view — Original, red above / blue below threshold
    ax5 = fig.add_subplot(2, 4, 5)
    if np.any(above_mask_orig):
        ax5.scatter(u_orig[above_mask_orig], v_orig[above_mask_orig],
                    c='red', s=30, alpha=0.6, label=f'Above {dB}dB threshold')
    if np.any(below_mask_orig):
        ax5.scatter(u_orig[below_mask_orig], v_orig[below_mask_orig],
                    c='blue', s=30, alpha=0.6, label=f'Below {dB}dB threshold')
    ax5.set_xlabel('U (Primary plane direction)')
    ax5.set_ylabel('V (Secondary plane direction)')
    ax5.set_title('In-Plane View - Original')
    ax5.axis('equal')
    ax5.legend(fontsize=7)
    ax5.grid(True, alpha=0.3)

    # 6. In-plane view — Filtered, red above / blue below, with radial edge
    ax6 = fig.add_subplot(2, 4, 6)
    if np.any(above_mask_filt):
        ax6.scatter(u_filt[above_mask_filt], v_filt[above_mask_filt],
                    c='red', s=30, alpha=0.6, label=f'Above {dB}dB threshold')
    if np.any(below_mask_filt):
        ax6.scatter(u_filt[below_mask_filt], v_filt[below_mask_filt],
                    c='blue', s=30, alpha=0.6, label=f'Below {dB}dB threshold')
    ax6.plot(edge_u_closed, edge_v_closed, 'k-', linewidth=2, label=f'{dB}dB edge')
    ax6.scatter(0, 0, color='yellow', s=150, marker='*', label='Geometric center', zorder=5)

    # Draw cardinal radius lines
    r0   = cardinal_dims['r_0']
    r90  = cardinal_dims['r_90']
    r180 = cardinal_dims['r_180']
    r270 = cardinal_dims['r_270']
    if not np.isnan(r0):
        ax6.annotate('', xy=(r0, 0), xytext=(0, 0),
                     arrowprops=dict(arrowstyle='->', color='cyan', lw=1.5))
    if not np.isnan(r180):
        ax6.annotate('', xy=(-r180, 0), xytext=(0, 0),
                     arrowprops=dict(arrowstyle='->', color='cyan', lw=1.5))
    if not np.isnan(r90):
        ax6.annotate('', xy=(0, r90), xytext=(0, 0),
                     arrowprops=dict(arrowstyle='->', color='magenta', lw=1.5))
    if not np.isnan(r270):
        ax6.annotate('', xy=(0, -r270), xytext=(0, 0),
                     arrowprops=dict(arrowstyle='->', color='magenta', lw=1.5))

    ax6.set_xlabel('U (Primary plane direction)')
    ax6.set_ylabel('V (Secondary plane direction)')
    ax6.set_title(f'In-Plane View — {dB}dB Edge\n'
                  f'W={cardinal_dims["width"]:.4f}  H={cardinal_dims["height"]:.4f}')
    ax6.axis('equal')
    ax6.legend(fontsize=7)
    ax6.grid(True, alpha=0.3)

    # 7. Radial amplitude profile
    ax7 = fig.add_subplot(2, 4, 7)
    point_radii_all = np.sqrt(u_orig**2 + v_orig**2)
    ax7.scatter(point_radii_all[above_mask_orig], amp_orig[above_mask_orig],
                s=10, alpha=0.4, c='red', label=f'Above {dB}dB threshold')
    ax7.scatter(point_radii_all[below_mask_orig], amp_orig[below_mask_orig],
                s=10, alpha=0.4, c='blue', label=f'Below {dB}dB threshold')
    ax7.axhline(center_amplitude, color='r', linestyle='--', linewidth=2,
                label=f'Center amplitude: {center_amplitude:.3f}')
    ax7.axhline(threshold, color='orange', linestyle='--', linewidth=2,
                label=f'{dB}dB threshold: {threshold:.3f}')
    ax7.set_xlabel('Distance from geometric center')
    ax7.set_ylabel('Normalized Amplitude')
    ax7.set_title('Radial Amplitude Profile')
    ax7.legend(fontsize=7)
    ax7.grid(True, alpha=0.3)

    # 8. Polar edge profile
    ax8 = fig.add_subplot(2, 4, 8, projection='polar')
    ax8.plot(edge_angles, edge_radii, 'r-', linewidth=2)
    ax8.fill(edge_angles, edge_radii, alpha=0.2, color='red')
    for angle_deg, r_val, color in [
        (0,   r0,   'cyan'),
        (90,  r90,  'magenta'),
        (180, r180, 'cyan'),
        (270, r270, 'magenta'),
    ]:
        if not np.isnan(r_val):
            ax8.plot(np.radians(angle_deg), r_val, 'o', color=color, markersize=8)
    ax8.set_title(f'{dB}dB Edge Profile\n(Polar)', pad=15)

    plt.suptitle(f'{defect} — {dB}dB Radial Edge (neighbour-confirmed)', fontsize=13, fontweight='bold')
    plt.tight_layout()

    output_path = f'{base_path}radial_edge_visualization_defect_{defect}.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Visualization saved to: {output_path}")
    plt.close()

if __name__ == "__main__":
    main()