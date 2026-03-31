"""
extract_circles.py

Extracts circular clusters from a PLY or PCD point cloud by Z depth range,
then saves each cluster as a binary PLY file with all fields preserved.

Requirements:
    pip install numpy scipy
"""

import numpy as np
from scipy.spatial import cKDTree
import os
import argparse

# ======================================================
#  EDIT THESE VALUES
# ======================================================

Z_MIN             = 0.0001   # minimum Z depth to keep
Z_MAX             = 0.05   # maximum Z depth to keep
CIRCLE_DIAMETER   = 0.010    # expected circle diameter in metres (10 mm)
SIZE_TOLERANCE    = 0.5      # accept clusters within this fraction of diameter (0.5 = +/-50%)
EXPECTED_CLUSTERS = 10       # how many circles to find
MIN_SAMPLES       = 10       # minimum points per cluster
VOXEL_SIZE        = 0.001    # downsampling voxel size (keep small relative to circle)
GAIN              = 173      # used in output filename
TVG               = 4        # used in output filename
OUTPUT_DIR        = "/home/rebecca/ut_servoing/src/peak_ros/src/peak_ros/bags/New_angle/Corr/Gate_5/"

# Derived from CIRCLE_DIAMETER -- do not edit these directly
EPS      = CIRCLE_DIAMETER * 0.19
SIZE_MIN = 0.005
SIZE_MAX = 0.025

# ======================================================


# ---- PLY reader ----------------------------------------------------------

def read_ply(path):
    fields = []
    data_type = "ascii"
    n_vertices = 0
    header_bytes = 0

    with open(path, "rb") as f:
        while True:
            raw_line = f.readline()
            header_bytes += len(raw_line)
            line = raw_line.decode("utf-8", errors="replace").strip()
            if line.startswith("property float") or line.startswith("property double"):
                fields.append(line.split()[-1])
            elif line.startswith("element vertex"):
                n_vertices = int(line.split()[-1])
            elif "binary" in line:
                data_type = "binary"
            elif line == "end_header":
                break

    print(f"  PLY fields : {fields}")
    print(f"  PLY format : {data_type}")
    print(f"  PLY points : {n_vertices:,}")

    n_fields = len(fields)
    if data_type == "binary":
        with open(path, "rb") as f:
            f.seek(header_bytes)
            raw = np.frombuffer(f.read(), dtype=np.float32)
        data = raw[: n_vertices * n_fields].reshape(n_vertices, n_fields).copy()
    else:
        with open(path, "rb") as f:
            f.seek(header_bytes)
            data = np.loadtxt(f, dtype=np.float32, max_rows=n_vertices)

    return data, fields


# ---- PCD reader ----------------------------------------------------------

def read_pcd(path):
    fields = []
    count = 0
    data_type = "ascii"
    header_bytes = 0

    with open(path, "rb") as f:
        for _ in range(50):
            raw_line = f.readline()
            header_bytes += len(raw_line)
            line = raw_line.decode("utf-8", errors="replace").strip()
            if line.lower().startswith("fields"):
                fields = line.split()[1:]
            elif line.lower().startswith("points"):
                count = int(line.split()[1])
            elif line.lower().startswith("data"):
                data_type = line.split()[1].lower()
                break

    print(f"  PCD fields : {fields}")
    print(f"  PCD format : {data_type}")
    print(f"  PCD points : {count:,}")

    n_fields = len(fields)
    if data_type == "binary":
        with open(path, "rb") as f:
            f.seek(header_bytes)
            raw = np.frombuffer(f.read(), dtype=np.float32)
        data = raw[: count * n_fields].reshape(count, n_fields).copy()
    else:
        with open(path, "rb") as f:
            f.seek(header_bytes)
            data = np.loadtxt(f, dtype=np.float32, max_rows=count)

    return data, fields


# ---- Auto reader ---------------------------------------------------------

def read_point_cloud(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".ply":
        return read_ply(path)
    elif ext == ".pcd":
        return read_pcd(path)
    else:
        raise ValueError(f"Unsupported format: {ext}  (expected .ply or .pcd)")


# ---- Binary PLY writer ---------------------------------------------------

def write_ply(path, data, fields):
    n = len(data)
    dtype = np.dtype([(name, np.float32) for name in fields])
    structured = np.empty(n, dtype=dtype)
    for i, name in enumerate(fields):
        structured[name] = data[:, i]

    header = "ply\nformat binary_little_endian 1.0\nelement vertex {}\n".format(n)
    for name in fields:
        header += "property float {}\n".format(name)
    header += "end_header\n"

    with open(path, "wb") as f:
        f.write(header.encode("utf-8"))
        f.write(structured.tobytes())


# ---- Voxel downsampling --------------------------------------------------

def voxel_downsample(points, voxel_size):
    voxel_ids = np.floor(points / voxel_size).astype(np.int32)
    _, unique_idx = np.unique(voxel_ids, axis=0, return_index=True)
    return points[unique_idx]


# ---- BFS clustering ------------------------------------------------------

def cluster_points(xy, eps, min_samples):
    """Correct DBSCAN-equivalent BFS clustering using cKDTree."""
    tree = cKDTree(xy)
    n = len(xy)
    labels = -np.ones(n, dtype=np.int32)
    visited = np.zeros(n, dtype=bool)
    current_label = 0

    for i in range(n):
        if visited[i]:
            continue
        visited[i] = True
        neighbours = tree.query_ball_point(xy[i], eps)
        if len(neighbours) < min_samples:
            continue  # noise point for now — may be claimed later by a cluster

        # Start a new cluster and BFS expand it
        labels[i] = current_label
        queue = list(neighbours)
        while queue:
            j = queue.pop()
            if not visited[j]:
                visited[j] = True
                new_nb = tree.query_ball_point(xy[j], eps)
                if len(new_nb) >= min_samples:
                    queue.extend(new_nb)
            if labels[j] == -1:
                labels[j] = current_label
        current_label += 1

    return labels


# ---- Main ----------------------------------------------------------------

def extract_circles(
    input_file,
    output_dir=OUTPUT_DIR,
    z_min=Z_MIN,
    z_max=Z_MAX,
    expected_clusters=EXPECTED_CLUSTERS,
    eps=EPS,
    size_min=SIZE_MIN,
    size_max=SIZE_MAX,
    min_samples=MIN_SAMPLES,
    voxel_size=VOXEL_SIZE,
    gain=GAIN,
    tvg=TVG,
):
    # 1. Load
    print(f"Loading: {input_file}")
    data, fields = read_point_cloud(input_file)

    if len(data) == 0:
        raise ValueError("Point cloud is empty.")

    try:
        xi, yi, zi = fields.index("x"), fields.index("y"), fields.index("z")
    except ValueError:
        raise ValueError(f"File must have x, y, z fields. Found: {fields}")

    # 2. Z filter
    z = data[:, zi]
    print(f"  Z range in file : [{z.min():.6f}, {z.max():.6f}]")
    print(f"  Filtering Z     : [{z_min:.6f}, {z_max:.6f}]")

    mask = (z >= z_min) & (z <= z_max)
    filtered = data[mask]
    print(f"  Points in Z band: {len(filtered):,}")

    if len(filtered) == 0:
        raise ValueError(
            "No points in Z range. Adjust Z_MIN / Z_MAX at the top of the script.\n"
            f"(File Z range is [{z.min():.6f}, {z.max():.6f}])"
        )

    # 3. Downsample -> cluster -> map back
    xy_full = filtered[:, [xi, yi]]

    print(f"\nDownsampling (voxel_size={voxel_size}) ...")
    down_xy = voxel_downsample(xy_full, voxel_size)
    print(f"  Downsampled to : {len(down_xy):,} points")

    print(f"Clustering (eps={eps:.4f} m = {eps*1000:.1f} mm, min_samples={min_samples}) ...")
    down_labels = cluster_points(down_xy, eps, min_samples)

    print("  Mapping labels back to full point set ...")
    tree = cKDTree(down_xy)
    _, ind = tree.query(xy_full, workers=-1)
    labels = down_labels[ind]

    unique_labels = set(labels) - {-1}
    n_found = len(unique_labels)
    print(f"  Raw clusters found : {n_found}")
    print(f"  Noise points       : {int(np.sum(labels == -1)):,}")

    if n_found == 0:
        raise RuntimeError("No clusters found. Try increasing EPS or lowering MIN_SAMPLES.")

    # 4. Filter clusters by physical size (using known circle diameter)
    print(f"\nFiltering clusters by size ({size_min*1000:.1f}-{size_max*1000:.1f} mm) ...")
    cluster_info = []
    rejected = 0
    for lbl in unique_labels:
        idx = np.where(labels == lbl)[0]
        pts = filtered[idx][:, [xi, yi]]
        span_x = pts[:, 0].max() - pts[:, 0].min()
        span_y = pts[:, 1].max() - pts[:, 1].min()
        diameter_est = (span_x + span_y) / 2.0
        if diameter_est < size_min or diameter_est > size_max:
            rejected += 1
            print(f"  Rejected (est. diam={diameter_est*1000:.1f} mm)")
            continue
        cx = pts[:, 0].mean()
        cluster_info.append((cx, lbl, idx, diameter_est))

    cluster_info.sort(key=lambda t: t[0])
    print(f"  Kept {len(cluster_info)}, rejected {rejected}")

    if len(cluster_info) == 0:
        raise RuntimeError(
            "No clusters passed the size filter. "
            "Try adjusting CIRCLE_DIAMETER or SIZE_TOLERANCE."
        )
    if len(cluster_info) != expected_clusters:
        print(f"\n  Warning: expected {expected_clusters} clusters but got {len(cluster_info)}.")
        print("  Adjust CIRCLE_DIAMETER, SIZE_TOLERANCE, EPS, or Z range.")

    # 5. Save each cluster individually and accumulate for combined file
    os.makedirs(output_dir, exist_ok=True)
    print(f"\nSaving {len(cluster_info)} clusters to '{output_dir}/' ...")

    all_cluster_data = []
    for rank, (cx, lbl, idx, diam) in enumerate(cluster_info, start=1):
        cluster_data = filtered[idx]
        out_path = os.path.join(output_dir, f"Defect_{rank:02d}_{gain}Gain_{tvg}TVG.ply")
        write_ply(out_path, cluster_data, fields)
        print(f"  Defect_{rank:02d}_{gain}Gain_{tvg}TVG.ply"
              f" -- {len(cluster_data):,} pts"
              f" (X={cx:.4f}, est. diam={diam*1000:.1f} mm)")
        all_cluster_data.append(cluster_data)

    # 6. Save combined PLY with all defects
    combined = np.vstack(all_cluster_data)
    combined_path = os.path.join(output_dir, f"Defects_ALL_{gain}Gain_{tvg}TVG.ply")
    write_ply(combined_path, combined, fields)
    print(f"\nCombined: Defects_ALL_{gain}Gain_{tvg}TVG.ply -- {len(combined):,} pts total")

    print("\nDone!")


# ---- CLI -----------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Extract circular clusters from a PLY/PCD, preserving all fields."
    )
    parser.add_argument("input_file", help="Path to input .ply or .pcd file")
    parser.add_argument("--output-dir",        default=OUTPUT_DIR)
    parser.add_argument("--z-min",             type=float, default=Z_MIN)
    parser.add_argument("--z-max",             type=float, default=Z_MAX)
    parser.add_argument("--circle-diameter",   type=float, default=CIRCLE_DIAMETER,
                        help="Expected circle diameter in metres (default: 0.010 = 10 mm)")
    parser.add_argument("--size-tolerance",    type=float, default=SIZE_TOLERANCE,
                        help="Fractional size tolerance, e.g. 0.5 = +/-50%%")
    parser.add_argument("--expected-clusters", type=int,   default=EXPECTED_CLUSTERS)
    parser.add_argument("--min-samples",       type=int,   default=MIN_SAMPLES)
    parser.add_argument("--voxel-size",        type=float, default=VOXEL_SIZE)
    parser.add_argument("--gain",              type=int,   default=GAIN)
    parser.add_argument("--tvg",               type=int,   default=TVG)

    args = parser.parse_args()

    # Recompute derived values if overridden on CLI
    eps      = args.circle_diameter * 0.19
    size_min = 0.005
    size_max = 0.025

    extract_circles(
        input_file=args.input_file,
        output_dir=args.output_dir,
        z_min=args.z_min,
        z_max=args.z_max,
        expected_clusters=args.expected_clusters,
        eps=eps,
        size_min=size_min,
        size_max=size_max,
        min_samples=args.min_samples,
        voxel_size=args.voxel_size,
        gain=args.gain,
        tvg=args.tvg,
    )