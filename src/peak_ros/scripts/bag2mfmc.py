#!/usr/bin/env python3
"""
Convert ROS bag A-scan messages to MFMC format.
Uses the existing bag parsing functions to extract data.
"""

import rosbag
import struct
import numpy as np
import pandas as pd
import argparse
from pathlib import Path


def ascans_bag2dataframe(bag):
    """Extract A-scan data from bag messages."""
    a_scans = [[msg.count,
                msg.test_number,
                msg.dof,
                msg.channel,
                msg.amplitudes] for msg in bag]
    a_scans = pd.DataFrame(a_scans, columns=['Count',
                                              'Test Number',
                                              'Data Output Format',
                                              'Channel',
                                              'Amplitudes'])
    return a_scans


def observation_bag2dataframe(bag, start_time=None, end_time=None):
    """Extract observation data from bag file."""
    observation = [[t.to_sec(),
                    msg.header.stamp.to_sec(),
                    msg.header.frame_id,
                    msg.dof,
                    msg.gate_start,
                    msg.gate_end,
                    msg.ascan_length,
                    msg.num_ascans,
                    msg.digitisation_rate,
                    msg.n_focals,
                    msg.element_pitch,
                    msg.inter_element_spacing,
                    msg.vel_wedge,
                    msg.vel_couplant,
                    msg.vel_material,
                    msg.wedge_angle,
                    msg.wedge_depth,
                    msg.couplant_depth,
                    msg.specimen_depth,
                    ascans_bag2dataframe(msg.ascans),
                    msg.max_amplitude] for (topic, msg, t) in bag.read_messages(topics=['/peak/a_scans'], start_time=start_time, end_time=end_time)]
    observation = pd.DataFrame(observation, columns=['ROS Time Recorded (s)',
                                                     'ROS Time Sent (s)',
                                                     'Frame ID',
                                                     'Data Output Format',
                                                     'Gate Start',
                                                     'Gate End',
                                                     'A Scan Length',
                                                     'Number of A Scans',
                                                     'Digitisation Rate (Mhz)',
                                                     'Number of Focal Laws',
                                                     'Element Pitch (mm)',
                                                     'Inter Element Spacing (mm)',
                                                     'Wedge Velocity (m/s)',
                                                     'Couplant Velocity (m/s)',
                                                     'Material Velocity (m/s)',
                                                     'Wedge Angle (deg)',
                                                     'Wedge Depth (mm)',
                                                     'Couplant Depth (mm)',
                                                     'Specimen Depth (mm)',
                                                     'A Scans',
                                                     'Max Amplitude'])
    return observation


def write_mfmc(observation_df, output_path):
    """
    Write observation data to MFMC format.
    
    Structure:
    1. Header with magic number and version
    2. Metadata (constants - stored once)
    3. Per-message data (timestamps, frame_id)
    4. A-scan data arrays
    """
    
    output_path = Path(output_path)
    
    # Get the first observation to extract constant metadata
    first_obs = observation_df.iloc[0]
    
    # Extract constant metadata (values that don't change across observations)
    metadata = {
        'data_output_format': int(first_obs['Data Output Format']),
        'gate_start': float(first_obs['Gate Start']),
        'gate_end': float(first_obs['Gate End']),
        'ascan_length': int(first_obs['A Scan Length']),
        'num_ascans': int(first_obs['Number of A Scans']),
        'digitisation_rate': float(first_obs['Digitisation Rate (Mhz)']),
        'n_focals': int(first_obs['Number of Focal Laws']),
        'element_pitch': float(first_obs['Element Pitch (mm)']),
        'inter_element_spacing': float(first_obs['Inter Element Spacing (mm)']),
        'vel_wedge': float(first_obs['Wedge Velocity (m/s)']),
        'vel_couplant': float(first_obs['Couplant Velocity (m/s)']),
        'vel_material': float(first_obs['Material Velocity (m/s)']),
        'wedge_angle': float(first_obs['Wedge Angle (deg)']),
        'wedge_depth': float(first_obs['Wedge Depth (mm)']),
        'couplant_depth': float(first_obs['Couplant Depth (mm)']),
        'specimen_depth': float(first_obs['Specimen Depth (mm)']),
    }
    
    num_observations = len(observation_df)
    
    print(f"Writing MFMC file with:")
    print(f"  - {num_observations} observations")
    print(f"  - {metadata['num_ascans']} A-scans per observation")
    print(f"  - {metadata['ascan_length']} samples per A-scan")
    
    with open(output_path, 'wb') as f:
        # =====================================================================
        # HEADER SECTION
        # =====================================================================
        # Magic number and version
        f.write(b'MFMC')  # Magic number (4 bytes)
        f.write(struct.pack('<I', 2))  # Version 2 (4 bytes)
        f.write(struct.pack('<I', num_observations))  # Number of observations (4 bytes)
        
        # =====================================================================
        # CONSTANT METADATA SECTION (stored once)
        # =====================================================================
        f.write(struct.pack('<i', metadata['data_output_format']))
        f.write(struct.pack('<f', metadata['gate_start']))
        f.write(struct.pack('<f', metadata['gate_end']))
        f.write(struct.pack('<I', metadata['ascan_length']))
        f.write(struct.pack('<I', metadata['num_ascans']))
        f.write(struct.pack('<f', metadata['digitisation_rate']))
        f.write(struct.pack('<I', metadata['n_focals']))
        f.write(struct.pack('<f', metadata['element_pitch']))
        f.write(struct.pack('<f', metadata['inter_element_spacing']))
        f.write(struct.pack('<f', metadata['vel_wedge']))
        f.write(struct.pack('<f', metadata['vel_couplant']))
        f.write(struct.pack('<f', metadata['vel_material']))
        f.write(struct.pack('<f', metadata['wedge_angle']))
        f.write(struct.pack('<f', metadata['wedge_depth']))
        f.write(struct.pack('<f', metadata['couplant_depth']))
        f.write(struct.pack('<f', metadata['specimen_depth']))
        
        # =====================================================================
        # PER-OBSERVATION DATA SECTION
        # =====================================================================
        for idx, row in observation_df.iterrows():
            # Write timestamps
            f.write(struct.pack('<d', row['ROS Time Recorded (s)']))
            f.write(struct.pack('<d', row['ROS Time Sent (s)']))
            
            # Write frame_id as a fixed-length string (64 bytes)
            frame_id = str(row['Frame ID']).encode('utf-8')
            frame_id = frame_id[:64].ljust(64, b'\0')  # Pad or truncate to 64 bytes
            f.write(frame_id)
        
        # =====================================================================
        # A-SCAN DATA SECTION
        # =====================================================================
        for idx, row in observation_df.iterrows():
            ascans_df = row['A Scans']
            
            # Sort by channel to ensure consistent ordering
            ascans_df = ascans_df.sort_values('Channel')
            
            # Write each A-scan
            for _, ascan_row in ascans_df.iterrows():
                # Write A-scan metadata
                f.write(struct.pack('<I', ascan_row['Count']))
                f.write(struct.pack('<I', ascan_row['Test Number']))
                f.write(struct.pack('<i', ascan_row['Data Output Format']))
                f.write(struct.pack('<I', ascan_row['Channel']))
                
                # Write amplitude data
                amplitudes = np.array(ascan_row['Amplitudes'], dtype=np.float32)
                
                # Ensure correct length
                if len(amplitudes) < metadata['ascan_length']:
                    amplitudes = np.pad(amplitudes, 
                                       (0, metadata['ascan_length'] - len(amplitudes)),
                                       mode='constant')
                elif len(amplitudes) > metadata['ascan_length']:
                    amplitudes = amplitudes[:metadata['ascan_length']]
                
                f.write(amplitudes.tobytes())
    
    file_size = output_path.stat().st_size
    print(f"\nWrote MFMC file: {output_path}")
    print(f"File size: {file_size:,} bytes")
    
    return True


def read_mfmc_header(mfmc_path):
    """
    Read and display the header and metadata from an MFMC file.
    Useful for verifying the file was written correctly.
    """
    with open(mfmc_path, 'rb') as f:
        # Read header
        magic = f.read(4)
        version = struct.unpack('<I', f.read(4))[0]
        num_observations = struct.unpack('<I', f.read(4))[0]
        
        print(f"MFMC File Header:")
        print(f"  Magic: {magic}")
        print(f"  Version: {version}")
        print(f"  Observations: {num_observations}")
        print()
        
        # Read metadata
        data_output_format = struct.unpack('<i', f.read(4))[0]
        gate_start = struct.unpack('<f', f.read(4))[0]
        gate_end = struct.unpack('<f', f.read(4))[0]
        ascan_length = struct.unpack('<I', f.read(4))[0]
        num_ascans = struct.unpack('<I', f.read(4))[0]
        digitisation_rate = struct.unpack('<f', f.read(4))[0]
        n_focals = struct.unpack('<I', f.read(4))[0]
        element_pitch = struct.unpack('<f', f.read(4))[0]
        inter_element_spacing = struct.unpack('<f', f.read(4))[0]
        vel_wedge = struct.unpack('<f', f.read(4))[0]
        vel_couplant = struct.unpack('<f', f.read(4))[0]
        vel_material = struct.unpack('<f', f.read(4))[0]
        wedge_angle = struct.unpack('<f', f.read(4))[0]
        wedge_depth = struct.unpack('<f', f.read(4))[0]
        couplant_depth = struct.unpack('<f', f.read(4))[0]
        specimen_depth = struct.unpack('<f', f.read(4))[0]
        
        print(f"Constant Metadata:")
        print(f"  Data Output Format: {data_output_format}")
        print(f"  Gate Start: {gate_start}")
        print(f"  Gate End: {gate_end}")
        print(f"  A-scan Length: {ascan_length}")
        print(f"  Num A-scans per observation: {num_ascans}")
        print(f"  Digitisation Rate: {digitisation_rate} MHz")
        print(f"  Num Focal Laws: {n_focals}")
        print(f"  Element Pitch: {element_pitch} mm")
        print(f"  Inter Element Spacing: {inter_element_spacing} mm")
        print(f"  Wedge Velocity: {vel_wedge} m/s")
        print(f"  Couplant Velocity: {vel_couplant} m/s")
        print(f"  Material Velocity: {vel_material} m/s")
        print(f"  Wedge Angle: {wedge_angle} deg")
        print(f"  Wedge Depth: {wedge_depth} mm")
        print(f"  Couplant Depth: {couplant_depth} mm")
        print(f"  Specimen Depth: {specimen_depth} mm")


def main():
    parser = argparse.ArgumentParser(
        description='Convert ROS bag A-scan data to MFMC format',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Convert entire bag
  python rosbag_to_mfmc.py input.bag output.mfmc
  
  # Verify the output
  python rosbag_to_mfmc.py output.mfmc --verify
        """
    )
    
    parser.add_argument('input', help='Input ROS bag file or MFMC file (for --verify)')
    parser.add_argument('output', nargs='?', help='Output MFMC file (not needed for --verify)')
    parser.add_argument('--verify', action='store_true', 
                       help='Read and display MFMC file header/metadata')
    parser.add_argument('--start-time', type=float, 
                       help='Start time in seconds (optional)')
    parser.add_argument('--end-time', type=float,
                       help='End time in seconds (optional)')
    
    args = parser.parse_args()
    
    if args.verify:
        # Verify mode - read and display MFMC header
        read_mfmc_header(args.input)
        return 0
    
    if not args.output:
        parser.error("output file required (unless using --verify)")
    
    print(f"Opening ROS bag: {args.input}")
    
    with rosbag.Bag(args.input, 'r') as bag:
        # Get bag info
        info = bag.get_type_and_topic_info()
        print(f"\nTopics in bag:")
        for topic, topic_info in info.topics.items():
            print(f"  {topic}: {topic_info.message_count} messages ({topic_info.msg_type})")
        
        # Parse the bag
        print(f"\nParsing observations from /peak/a_scans...")
        observation_df = observation_bag2dataframe(bag, 
                                                   start_time=args.start_time,
                                                   end_time=args.end_time)
        
        print(f"Extracted {len(observation_df)} observations")
        
        if len(observation_df) == 0:
            print("ERROR: No observations found!")
            return 1
        
        # Write to MFMC
        success = write_mfmc(observation_df, args.output)
        
        if success:
            print("\n" + "="*60)
            print("Conversion complete!")
            print("="*60)
            return 0
        else:
            return 1


if __name__ == '__main__':
    exit(main())