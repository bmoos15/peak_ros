"""
PLY Point Cloud Analysis Tool
==============================
Analyses PLY files containing x, y, z, time_of_flight, and amplitude fields.

Steps performed:
  1. Fit a plane to the point cloud and report tilt angles (roll/pitch from horizontal)
  2. Find the geometric centre of the point cloud
  3. Average the 20 highest-amplitude points
  4. March outward in the 4 cardinal directions (5 ray-fans each, ±10° spread)
     from the plane centre and locate the -6 dB boundary
  5. Compute the in-plane width (left–right) and height (up–down) of the indication
"""

import sys
import struct
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from plyfile import PlyData, PlyElement
from scipy.optimize import least_squares
from typing import Optional


# ---------------------------------------------------------------------------
# Tee: mirrors stdout to both console and a log file
# ---------------------------------------------------------------------------

class Tee:
    """Write all print() output to both the terminal and a log file."""

    def __init__(self, filepath):
        self._terminal = sys.stdout
        self._log = open(filepath, "w", encoding="utf-8")

    def write(self, message):
        self._terminal.write(message)
        self._log.write(message)

    def flush(self):
        self._terminal.flush()
        self._log.flush()

    def close(self):
        sys.stdout = self._terminal   # restore original stdout
        self._log.close()


# ---------------------------------------------------------------------------
# 1. PLY reader (header + binary / ASCII body)
# ---------------------------------------------------------------------------

def read_ply(filepath):
    """Return an (N, 5) array [x, y, z, tof, amplitude] from a PLY file."""
    path = Path(filepath)

    with open(path, "rb") as f:
        # --- parse header ---------------------------------------------------
        plydata = PlyData.read(path)
        vertex = plydata['vertex']

        x = vertex['x']
        y = vertex['y']
        z = vertex['z']
        amps = vertex['scalar_Amplitudes']
        
        data = np.column_stack([x, y, z, amps])

    return data


# ---------------------------------------------------------------------------
# 2. Plane fitting (SVD / PCA)
# ---------------------------------------------------------------------------
def fit_plane_svd(data):
    """
    Fit a plane to a point cloud using SVD (least-squares best fit).

    Returns:
        centroid: geometric center of the point cloud (3,)
        normal:   unit normal vector of the fitted plane (3,)
    """
    points = np.array(data[:,:3])


    window = 20
    # smoothed = np.convolve(data[:, 3], np.ones(window)/window, mode='valid')
    # max_amp_index = np.argmax(smoothed) + window // 2
    max_amp_index = np.argpartition(data[:, 3], -window)[-window:]

    # centroid = np.mean(points, axis=0)
    centroid = np.mean(points[max_amp_index], axis=0)
    centered = points - centroid

    # SVD of the centered points — the normal is the left singular vector
    # corresponding to the smallest singular value
    _, _, Vt = np.linalg.svd(centered)
    normal = Vt[-1]  # last row = direction of least variance = plane normal

    return centroid, normal

# ---------------------------------------------------------------------------
# 3. Get rotation matrix from normal using Rodrigues' Formula
# ---------------------------------------------------------------------------
def build_rotation_matrix(normal):
    """
    Build a rotation matrix that aligns the given normal vector to the Z-axis.
    Uses Rodrigues' rotation formula.

    Returns:
        R: (3, 3) rotation matrix
    """
    normal = normal / np.linalg.norm(normal)
    z_axis = np.array([0.0, 0.0, 1.0])

    cross = np.cross(normal, z_axis)
    cross_norm = np.linalg.norm(cross)

    # Already aligned (or anti-aligned) with Z
    if cross_norm < 1e-6:
        return np.eye(3) if np.dot(normal, z_axis) > 0 else np.diag([1.0, -1.0, -1.0])

    axis = cross / cross_norm
    angle = np.arcsin(np.clip(cross_norm, -1.0, 1.0))
    if np.dot(normal, z_axis) < 0:
        angle = np.pi - angle

    # Rodrigues' rotation formula
    K = np.array([
        [    0.0, -axis[2],  axis[1]],
        [ axis[2],     0.0, -axis[0]],
        [-axis[1],  axis[0],     0.0]
    ])
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    
    return R

# ---------------------------------------------------------------------------
# 4. Transform data
# ---------------------------------------------------------------------------

def transform_to_plane_frame(data):
    """
    Transform a point cloud into a coordinate system where:
      - Origin  = geometric center of the point cloud
      - XY plane = best-fit plane through the points
      - Z-axis  = plane normal (points with positive Z are 'above' the plane)

    Args:
        points: (N, 3) array of point cloud coordinates

    Returns:
        transformed:  (N, 3) points in the new coordinate system
        centroid:     (3,)   original centroid (for inversion)
        R:            (3, 3) rotation matrix applied (for inversion)
    """
    points = np.array(data[:,:3])

    centroid, normal = fit_plane_svd(data)

    # 1. Translate so centroid is at origin
    centered_data = points - centroid

    # 2. Rotate so the fitted plane aligns with XY (normal aligns with Z)
    R = build_rotation_matrix(normal)
    transformed_data = (R @ centered_data.T).T

    return transformed_data, centroid, R

def transform_back(data, R, centroid):
    transformed_back_data =(R.T @ data.T).T + centroid

    return transformed_back_data

# ---------------------------------------------------------------------------
# 5. Calculate dB drop
# ---------------------------------------------------------------------------

def dB_drop(data, centroid, tolerance, dB, path, defect_idx, x_yn):
    """
    Within a band around centroid[0], march in increasing Y and plot points
    where amplitude crosses the -6 dB threshold (half-power: max_amp / 2).

    Parameters:
        data          – Nx4 array [x, y, z, amplitude]
        centroid      – (x, y, z) centroid from fit_plane
        x_tolerance   – half-width of the X band around centroid[0]
    """
    n_samples = 1


    if x_yn:
        output_path = f'{path}Width_defect_{defect_idx+1}.png'
    else: 
        output_path = f'{path}Height_defect_{defect_idx+1}.png'

    x, y, z, amp = data[:, 0], data[:, 1], data[:, 2], data[:, 3]


    # _dB threshold drop
    max_amp = np.max(amp)
    threshold = (max_amp) / 10**(dB/20) 

    # max_amp = 32768
    # threshold = 0.80*32768

    # --- Band filter around centroid X ---
    if x_yn: 
        mask = np.abs(x) <= tolerance
    else:
        mask = np.abs(y) <= tolerance
    

    band = data[mask]

    if len(band) == 0:
        raise ValueError("No points found within tolerance of centroid.")

    if x_yn: 
        band = band[np.argsort(band[:, 1])]  # sort by Y ascending
    else:
        band = band[np.argsort(band[:, 0])]  # sort by X ascending


    bx, by, bz, bamp = band[:, 0], band[:, 1], band[:, 2], band[:, 3]

    # --- Find crossings via sign changes relative to threshold ---
    above = bamp >= threshold
    threshold_crossings = band[bamp >= threshold]
    crossings, crossings_amp = [], []

    for i in range(len(band) - 1):
        if above[i] != above[i + 1]:                      # sign change → crossing
            # Linear interpolation to find precise Y crossing
            a0, a1 = bamp[i], bamp[i + 1]
            
            if x_yn:
                dim0, dim1 = by[i], by[i+1]
            else:
                dim0, dim1 = bx[i], bx[i +1]

            t = (threshold - a0) / (a1 - a0)
            crossings.append(dim0 + t * (dim1 - dim0))
            crossings_amp.append(threshold)

    # print(crossings)
    # --- Plot ---
    fig, ax = plt.subplots(figsize=(9, 4))
   
    if x_yn:
        ax.plot(by, bamp, color="lightblue", lw=1.0, alpha=0.5, zorder=2)
        ax.scatter(by, bamp, color="steelblue", s=10, label="Amplitude (band)", zorder=3)
        ax.axhline(threshold, color="tomato", lw=1.2, ls="--",
               label=f"−6 dB threshold ({threshold:.3f})")
        ax.axhline(max_amp, color="gray", lw=0.8, ls=":",
               label=f"Max amp ({max_amp:.3f})")
        ax.set_xlabel('Y(m)')
    else:
        ax.plot(bx, bamp, color="orange", lw=1.0, alpha=0.5, zorder=2)
        ax.scatter(bx, bamp, color="yellow", s=10, label="Amplitude (band)", zorder=3)
        ax.axhline(threshold, color="tomato", lw=1.2, ls="--",
               label=f"−6 dB threshold ({threshold:.3f})")
        ax.axhline(max_amp, color="gray", lw=0.8, ls=":",
               label=f"Max amp ({max_amp:.3f})")
        ax.set_xlabel("X(m)")

    ax.set_ylabel("Amplitude")
    ax.set_title(f"−6 dB Crossings  |  Band: {centroid[0]:.4f} ± {tolerance:.4f}")
    ax.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')

    #TODO: Change this to be the second crossing once stable

    length = crossings[-1] - crossings[0]

    return length, threshold, threshold_crossings

def residuals(params, pts):
    cx, cy, r = params
    return np.sqrt((pts[:,0]-cx)**2 + (pts[:,1]-cy)**2) - r

def save_ply(diameters, dB, path, defect_idx, circle):
    diameters = np.vstack(diameters)

    if circle:
        name='Diameter'
    else:
        name = 'Circle'

    # dtype = [('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('amplitude', 'f4')]
    dtype_thresh_cross = [('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('red', 'u2'), ('green', 'u1'), ('blue', 'u1')]

    # # Main point cloud
    # records = np.array(
    #     [(row[0], row[1], row[2], row[3]) for row in combined_points], dtype=dtype)
    # PlyData([PlyElement.describe(records, 'vertex')], text=False).write(
    #     f'{path}{dB}dB{defect_idx+1}_filtered_z.ply')

    # Band width points
    records = np.array(
        [(row[0], row[1], row[2], 0, 0, 0) for row in diameters], dtype=dtype_thresh_cross)
    PlyData([PlyElement.describe(records, 'vertex')], text=False).write(
        f'{path}{name}_{dB}dB.ply')

    print(f"PLY successfully saved to {path}")

    # # Band height points
    # records_height = np.array(
    #     [(row[0], row[1], row[2], 0, 0, 0) for row in threshold_crossings_y], dtype=dtype_thresh_cross)
    # PlyData([PlyElement.describe(records_height, 'vertex')], text=False).write(
    #     f'{path}A{dB}dB{defect_idx+1}_height.ply')


def main():
    dB = 6
    Gain = 135
    TVG = 8

    spoiler = True

    amp_buffer = 0.1

    # x_tolerance = 0.0002
    # y_tolerance = 0.002

    num_angles = 4

    base_path = f'/home/rebecca/fast_ut_servoing/src/peak_ros/src/peak_ros/bags/manual_flat_with_encoder_angle/'

    # -----------------------------------------------------------------------
    # Redirect all print() output to both the terminal and a summary txt file
    # -----------------------------------------------------------------------
    summary_path = f'{base_path}Error_Summary_{Gain}Gain_{TVG}TVG.txt'
    tee = Tee(summary_path)
    sys.stdout = tee

    # defects = [f'Defect_01_{Gain}Gain_{TVG}TVG', f'Defect_02_{Gain}Gain_{TVG}TVG',
    #            f'Defect_03_{Gain}Gain_{TVG}TVG', f'Defect_04_{Gain}Gain_{TVG}TVG',
    #            f'Defect_05_{Gain}Gain_{TVG}TVG', f'Defect_06_{Gain}Gain_{TVG}TVG',
    #            f'Defect_07_{Gain}Gain_{TVG}TVG', f'Defect_08_{Gain}Gain_{TVG}TVG',
    #            f'Defect_09_{Gain}Gain_{TVG}TVG', f'Defect_10_{Gain}Gain_{TVG}TVG']

    # defects = [f'Defect_06_{Gain}Gain_{TVG}TVG']

    defects = [f'Defect_5', f'Defect_6']

    # defects = [f'Defect_01_{Gain}Gain_{TVG}TVG', f'Defect_02_{Gain}Gain_{TVG}TVG',
    #            f'Defect_03_{Gain}Gain_{TVG}TVG', f'Defect_04_{Gain}Gain_{TVG}TVG',
    #            f'Defect_05_{Gain}Gain_{TVG}TVG', f'Defect_06_{Gain}Gain_{TVG}TVG',
    #            f'Defect_07_{Gain}Gain_{TVG}TVG', f'Defect_08_{Gain}Gain_{TVG}TVG',
    #            f'Defect_09_{Gain}Gain_{TVG}TVG', f'Defect_10_{Gain}Gain_{TVG}TVG',
    #            f'Defect_11_{Gain}Gain_{TVG}TVG', f'Defect_12_{Gain}Gain_{TVG}TVG',
    #            f'Defect_13_{Gain}Gain_{TVG}TVG', f'Defect_14_{Gain}Gain_{TVG}TVG',
    #            f'Defect_15_{Gain}Gain_{TVG}TVG'
    #            ]    

    # defects = [f'Defect_01_{Gain}Gain_{TVG}TVG', f'Defect_02_{Gain}Gain_{TVG}TVG',
    #            f'Defect_03_{Gain}Gain_{TVG}TVG', f'Defect_04_{Gain}Gain_{TVG}TVG',
    #            f'Defect_05_{Gain}Gain_{TVG}TVG', f'Defect_06_{Gain}Gain_{TVG}TVG',
    #            f'Defect_07_{Gain}Gain_{TVG}TVG', f'Defect_08_{Gain}Gain_{TVG}TVG',
    #            f'Defect_09_{Gain}Gain_{TVG}TVG', f'Defect_10_{Gain}Gain_{TVG}TVG',
    #            f'Defect_11_{Gain}Gain_{TVG}TVG', f'Defect_12_{Gain}Gain_{TVG}TVG',
    #            ]

    consolidated_summary_file = f'{base_path}{dB}dB_radial_analysis_summary.txt'

    print(f"Processing Defects from {base_path}")

    diameters=[]
    circles_i_hope=[]
    diameters = []
    errors = []

    try:
        for defect_idx, defect in enumerate(defects):
            
            print(f"Processing Defect{defect_idx+1}")

            input_file = f'{base_path}{defect}.ply'
            data = read_ply(input_file)

            #Sort out the points with the same x, y, and lower z
            # data_high_z = filter_max_z_numpy(data)
            
            # Plane Fitting
            transformed_data, centroid, R = transform_to_plane_frame(data)
            # u, v = build_plane_basis(normal)

            #Add amplitude back in
            transformed_data_amp = np.column_stack([transformed_data, data[:, 3]])

            # width, amp_threshold, threshold_crossings_x = dB_drop(transformed_data_amp, centroid, x_tolerance, dB, base_path, defect_idx, x_yn = True)
            # height, not_used, threshold_crossings_y = dB_drop(transformed_data_amp, centroid, y_tolerance, dB, base_path, defect_idx, x_yn = False)

            amp_threshold = (np.max(data[:,3]) - np.min(data[:,3])) / 10**(dB/20) + np.min(data[:,3])
            circle_threshold = transformed_data_amp[(data[:, 3] >= (amp_threshold - (amp_buffer * amp_threshold))) & (data[:, 3] <= (amp_threshold + (amp_buffer*amp_threshold))), :3]

            #Fit Circle
            x0 = [circle_threshold[:,0].mean(), circle_threshold[:,1].mean(), 1.0]
            res = least_squares(residuals, x0, args=(circle_threshold,))
            cx, cy, average_diameter = res.x[0], res.x[1], res.x[2] * 2

            if spoiler and defect_idx in (0, 1, 2, 3, 4):
                target = 0.005
            elif spoiler and defect_idx in (5, 6, 7, 8, 9):
                target = 0.004
            else:
                target = 0.003


            error = (average_diameter - target)*1000
            
            print(f"Max_amplitude: {np.max(data[:,3]):0.3f}")
            print(f"Diameter: {(average_diameter*1000):0.5f}mm, Target: {(target*1000):.3f}mm, Error: {error:0.3f}mm")



            data_back = transform_back(transformed_data, R, centroid)
            circle_back = transform_back(circle_threshold[:,:3], R, centroid)

            # above_x = transform_back(threshold_crossings_x[:, :3], R, centroid)
            # above_y = transform_back(threshold_crossings_y[:, :3], R, centroid)

            transform_back_amp = np.column_stack([data_back, data[:,3]])
            # data_filtered = remove_drop(transformed_data, centroid, height, width, amp_threshold)
            
            # diameters.append(above_x)
            # diameters.append(above_y)
            diameters.append(average_diameter)
            errors.append(error)
            circles_i_hope.append(circle_back)

        save_ply(circles_i_hope, dB, base_path, defect_idx, False)
        print(f"Average_error: {np.mean(np.abs(errors)):.3f}mm, Max_error: {np.max(np.abs(errors)):.3f}mm, Range of Error: {np.max(errors)-np.min(errors):.3f}mm")
        # save_ply(diameters, dB, base_path, defect_idx, True)

    finally:
        # Always restore stdout and close the log file, even if an error occurs
        tee.close()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":

    main()