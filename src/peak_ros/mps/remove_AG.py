#!/usr/bin/env python3

import os

def remove_ag_lines(input_file, output_file=None):
    """
    Remove all lines that begin with 'AG.' from a text file.
    
    Args:
        input_file (str): Path to the input text file
        output_file (str): Path to the output file (optional)
                          If not provided, will overwrite the input file
    """
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Create full path to the input file
    input_path = os.path.join(script_dir, input_file)
    
    if output_file is None:
        output_path = input_path
    else:
        output_path = os.path.join(script_dir, output_file)
    
    try:
        # Read all lines from the input file
        with open(input_path, 'r', encoding='utf-8') as file:
            lines = file.readlines()
        
        # Filter out lines that begin with 'AG.'
        filtered_lines = [line for line in lines if not line.strip().startswith('AG.')]
        filtered_lines = [line for line in filtered_lines if not line.strip().startswith('GAT ')]
        filtered_lines = [line for line in filtered_lines if not line.strip().startswith('GAN ')]
        filtered_lines = [line for line in filtered_lines if not line.strip().startswith('TRM')]
        filtered_lines = [line for line in filtered_lines if not line.strip().startswith('TTD')]


        # Write the filtered lines to the output file
        with open(output_path, 'w', encoding='utf-8') as file:
            file.writelines(filtered_lines)
        
        print(f"Successfully processed {input_path}")
        print(f"Removed {len(lines) - len(filtered_lines)} lines beginning with 'AG.'")
        
    except FileNotFoundError:
        print(f"Error: File '{input_path}' not found.")
        print(f"Make sure the file is in the same folder as this script: {script_dir}")
    except Exception as e:
        print(f"Error processing file: {e}")

def choose_file():
    """Let user choose from available text files"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    text_files = [f for f in os.listdir(script_dir) if f.endswith('.mps')]
    
    if not text_files:
        print("No .mps files found in the script directory")
        return None
    
    print("Available text files:")
    for i, file in enumerate(text_files, 1):
        print(f"  {i}. {file}")
    
    while True:
        try:
            choice = int(input("Enter the number of the file to process: "))
            if 1 <= choice <= len(text_files):
                return text_files[choice - 1]
            else:
                print("Invalid choice. Please try again.")
        except ValueError:
            print("Please enter a valid number.")

# In your main section:
if __name__ == "__main__":
    selected_file = choose_file()
    if selected_file:
        print(f"Processing file: {selected_file}")
        remove_ag_lines(selected_file)