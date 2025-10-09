#!/usr/bin/env python3

import struct
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable
import matplotlib.animation as animation
import matplotlib.cm as cm

path = "/home/rebecca/ut_servoing/src/peak_ros/src/peak_ros/scripts/"

file = "alum_corr_b_scans2.csv"
tof_data = pd.read_csv(f'{path}{file}', sep=',')
tof_data = np.array(tof_data)
# tof_1 = pd.read_csv(f'{path}{file.split(".")[0]}_tof_data.csv', sep=',')
# file = "pp_full_1745929517747275.pcd"
# tof_2 = pd.read_csv(f'{path}{file.split(".")[0]}_tof_data.csv', sep=',')
# file = "pp_full_1745929597870828.pcd"
# tof_3 = pd.read_csv(f'{path}{file.split(".")[0]}_tof_data.csv', sep=',')
# file = "pp_full_1745929676195761.pcd"
# tof_4 = pd.read_csv(f'{path}{file.split(".")[0]}_tof_data.csv', sep=',')

# tof_data = pd.concat([tof_1, tof_2, tof_3, tof_4])

b_scan_data = []

for j in range(1):

    data = np.array(tof_data)[j][9]

    data = data[2:-1]

    # Handle the escaped byte sequences
    binary_data = data.encode('utf-8').decode('unicode_escape').encode('latin1')

    # Each record is 16 bytes (4 fields × 4 bytes each)
    record_size = 16
    num_records = len(binary_data) // record_size


    # Parse the data
    X = []
    Y = []
    Z = []
    Amplitudes = []

    for i in range(num_records):
        offset = i * record_size
        
        if offset + record_size <= len(binary_data):
            # Unpack as little-endian float32 values
            try:
                x = struct.unpack('<f', binary_data[offset:offset+4])[0]
                y = struct.unpack('<f', binary_data[offset+4:offset+8])[0] 
                z = struct.unpack('<f', binary_data[offset+8:offset+12])[0]
                amp = struct.unpack('<f', binary_data[offset+12:offset+16])[0]
                
                X.append(x)
                Y.append(y)
                Z.append(z)
                Amplitudes.append(amp)
                
            except struct.error as e:
                print(f"Error parsing record {i}: {e}")
                break

    # Create DataFrame
    df = pd.DataFrame({
        'x': X,
        'y': Y, 
        'z': Z,
        'Amplitudes': Amplitudes,
    })

    start_idx = int(num_records * 0.3)

    b_scan_data.append({
        'Y': Y[start_idx:],
        'Z': Z[start_idx:], 
        'Amplitudes': Amplitudes[start_idx:],
        'frame': j
    })


dpi = 300
fig, ax = plt.subplots(dpi=dpi, figsize=(6, 3))

def animate(frame_idx):
    try:
        ax.clear()
        data = b_scan_data[frame_idx]

        # Get the rainbow colormap and modify it
        rainbow = cm.get_cmap('rainbow')
        colors = rainbow(np.linspace(0, 1, 256))

        # Find the middle index and set it to white
        mid_idx = len(colors) // 2
        colors[mid_idx - int(0.1 * max(Amplitudes)): mid_idx + int(0.1 * max(Amplitudes))] = [1, 1, 1, 1]  # RGBA for white

        custom_cmap = mcolors.ListedColormap(colors)

        im = ax.tricontourf(Y, Z, Amplitudes, cmap=custom_cmap)
        ax.invert_yaxis()
        #fig.gca().set_aspect('equal')
        ax.set_title("B-Scan Amplitudes")
        ax.set_xlabel("Specimen Position [m]")
        ax.set_ylabel("Depth [mm]")
        divider = make_axes_locatable(ax)
        cax = divider.append_axes("right", size="5%", pad=0.05)
        cbar = plt.colorbar(im, 
                            cax=cax, 
                            label="Amplitudes [dB]",
                            ticks=np.arange(0.0, max(Amplitudes)),
                            boundaries=[0.0, max(Amplitudes)]
                            )

        # ax.axvline(b_scan_pos,
        #            color='grey',
        #            linestyle='--',
        #            linewidth=2,
        #            label=f"B-Scan Position ({str(round(b_scan_pos, 1))} m)"
        #            )
        # ax.legend(loc='upper right')
        plt.tight_layout()
        # # plt.show()
        # plt.savefig(f'{path}images/b_scan/alum_corr_reconstruction{j}.png', dpi=dpi)

        return [im]


    except Exception as e:
        print(f"Unable to plot ToF C Scan: {e}")
        pass



anim = animation.FuncAnimation(fig, animate, frames = len(tof_data), interval = 200)

# Save as video
output_path = f'{path}images/b_scan/alum_corr_reconstruction_video.mp4'
anim.save(output_path, writer='ffmpeg', fps=5, dpi=dpi)
plt.close()