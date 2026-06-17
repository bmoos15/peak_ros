import re

def parse_txf_delays(ag_file_content):
    """Extract TXF delays from AG file, grouped by sweep index."""
    txf_groups = {}
    
    for line in ag_file_content.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[0] == 'TXF':
            sweep_idx = int(parts[1])
            element_idx = int(parts[2])
            if sweep_idx not in txf_groups:
                txf_groups[sweep_idx] = {}
            txf_groups[sweep_idx][element_idx] = element_idx  # value is always 0 for the -1 sentinel
        elif len(parts) == 4 and parts[0] == 'TXF':
            sweep_idx = int(parts[1])
            element_idx = int(parts[2])
            delay_val = int(parts[3])
            if sweep_idx not in txf_groups:
                txf_groups[sweep_idx] = {}
            txf_groups[sweep_idx][element_idx] = delay_val
    
    return txf_groups

def parse_rxf_delays(ag_file_content):
    """Extract RXF delays from AG file, grouped by sweep index."""
    rxf_groups = {}
    
    for line in ag_file_content.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == 'RXF':
            sweep_idx = int(parts[1])
            element_idx = int(parts[2])
            delay_val = int(parts[3])
            if sweep_idx not in rxf_groups:
                rxf_groups[sweep_idx] = {}
            rxf_groups[sweep_idx][element_idx] = delay_val
        elif len(parts) == 5 and parts[0] == 'RXF':
            sweep_idx = int(parts[1])
            element_idx = int(parts[2])
            delay_val = int(parts[3])
            if sweep_idx not in rxf_groups:
                rxf_groups[sweep_idx] = {}
            rxf_groups[sweep_idx][element_idx] = delay_val
    
    return rxf_groups

def merge_files(ag_content, flat_content):
    """Replace TXF/RXF delay values in flat file with those from AG file."""
    
    # Parse delays from AG file
    txf_delays = {}
    rxf_delays = {}
    
    for line in ag_content.splitlines():
        parts = line.split()
        
        if parts and parts[0] == 'TXF' and len(parts) == 4:
            sweep_idx = int(parts[1])
            element_idx = int(parts[2])
            delay_val = int(parts[3])
            key = (sweep_idx, element_idx)
            txf_delays[key] = delay_val
            
        elif parts and parts[0] == 'RXF' and len(parts) == 5:
            sweep_idx = int(parts[1])
            element_idx = int(parts[2])
            delay_val = int(parts[3])
            key = (sweep_idx, element_idx)
            rxf_delays[key] = delay_val
    
    # Now rewrite the flat file substituting delay values
    output_lines = []
    
    for line in flat_content.splitlines():
        parts = line.split()
        
        if parts and parts[0] == 'TXF' and len(parts) == 4:
            sweep_idx = int(parts[1])
            element_idx = int(parts[2])
            key = (sweep_idx, element_idx)
            if key in txf_delays:
                new_delay = txf_delays[key]
                output_lines.append(f'TXF {sweep_idx} {element_idx} {new_delay}')
            else:
                output_lines.append(line)
                
        elif parts and parts[0] == 'RXF' and len(parts) == 5:
            sweep_idx = int(parts[1])
            element_idx = int(parts[2])
            apod_val = parts[4]  # preserve apodisation value
            key = (sweep_idx, element_idx)
            if key in rxf_delays:
                new_delay = rxf_delays[key]
                output_lines.append(f'RXF {sweep_idx} {element_idx} {new_delay} {apod_val}')
            else:
                output_lines.append(line)
        else:
            output_lines.append(line)
    
    return '\n'.join(output_lines)


# --- Main ---
ag_file_path   = 'ArrayGen_Focal_Depth33_5.mps'    # Document 1 (with AG. headers and focal delays)
flat_file_path = 'immersion_128el_20aperture_60delay_orig.mps'  # Document 2 (all zeros, to be updated)
output_path    = 'merged_128el_20aperture_33_5delay.mps'

with open(ag_file_path, 'r') as f:
    ag_content = f.read()

with open(flat_file_path, 'r') as f:
    flat_content = f.read()

merged = merge_files(ag_content, flat_content)

with open(output_path, 'w') as f:
    f.write(merged)

print(f"Done. Merged file written to '{output_path}'")