#!/usr/bin/env python3
"""
PLY Metadata Checker
Analyzes PLY files to check their structure and identify why they might fail in Noether
"""

import sys
import struct

def parse_ply_header(filename):
    """Parse PLY header and return metadata"""
    metadata = {
        'format': None,
        'elements': {},
        'properties': {},
        'comments': [],
        'has_vertices': False,
        'has_faces': False,
        'vertex_count': 0,
        'face_count': 0
    }
    
    with open(filename, 'rb') as f:
        # Check magic number
        first_line = f.readline().decode('ascii').strip()
        if first_line != 'ply':
            return None, "Not a valid PLY file (missing 'ply' magic number)"
        
        current_element = None
        
        while True:
            line = f.readline().decode('ascii').strip()
            
            if line == 'end_header':
                break
                
            parts = line.split()
            if not parts:
                continue
                
            if parts[0] == 'format':
                metadata['format'] = ' '.join(parts[1:])
                
            elif parts[0] == 'comment':
                metadata['comments'].append(' '.join(parts[1:]))
                
            elif parts[0] == 'element':
                element_name = parts[1]
                element_count = int(parts[2])
                metadata['elements'][element_name] = element_count
                metadata['properties'][element_name] = []
                current_element = element_name
                
                if element_name == 'vertex':
                    metadata['has_vertices'] = True
                    metadata['vertex_count'] = element_count
                elif element_name == 'face':
                    metadata['has_faces'] = True
                    metadata['face_count'] = element_count
                    
            elif parts[0] == 'property' and current_element:
                metadata['properties'][current_element].append(' '.join(parts[1:]))
    
    return metadata, None

def print_metadata(filename, metadata):
    """Pretty print metadata"""
    print(f"\n{'='*60}")
    print(f"File: {filename}")
    print(f"{'='*60}")
    
    print(f"\nFormat: {metadata['format']}")
    
    if metadata['comments']:
        print(f"\nComments:")
        for comment in metadata['comments']:
            print(f"  - {comment}")
    
    print(f"\nElements:")
    for element, count in metadata['elements'].items():
        print(f"  {element}: {count}")
    
    print(f"\nProperties by element:")
    for element, props in metadata['properties'].items():
        print(f"\n  {element}:")
        for prop in props:
            print(f"    - {prop}")
    
    print(f"\n{'─'*60}")
    print(f"NOETHER COMPATIBILITY CHECK:")
    print(f"{'─'*60}")
    print(f"Has vertices: {metadata['has_vertices']} ({metadata['vertex_count']} vertices)")
    print(f"Has faces: {metadata['has_faces']} ({metadata['face_count']} faces)")
    
    if not metadata['has_faces'] or metadata['face_count'] == 0:
        print("\n⚠️  WARNING: This file has NO FACES!")
        print("   Noether requires mesh faces (polygons) to subdivide.")
        print("   This file appears to contain only a point cloud.")
        print("   You need to reconstruct a mesh surface from the points.")
    else:
        print("\n✓  This file has faces and should work with Noether")
    
    print(f"{'='*60}\n")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 ply_metadata_checker.py <file1.ply> [file2.ply] ...")
        print("\nThis tool checks PLY file metadata to diagnose Noether compatibility issues.")
        sys.exit(1)
    
    for filename in sys.argv[1:]:
        try:
            metadata, error = parse_ply_header(filename)
            if error:
                print(f"\nError reading {filename}: {error}")
                continue
            
            print_metadata(filename, metadata)
            
        except FileNotFoundError:
            print(f"\nError: File '{filename}' not found")
        except Exception as e:
            print(f"\nError processing {filename}: {str(e)}")

if __name__ == "__main__":
    main()