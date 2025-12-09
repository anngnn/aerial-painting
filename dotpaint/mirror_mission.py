#!/usr/bin/env python3
"""
Mirror X-axis movements in a drone mission file.
Usage: python mirror_mission.py input_mission.txt output_mission.txt
"""

import sys
import re

def mirror_x_axis(mission_text):
    """
    Mirror all X-axis movements in the mission by flipping their signs.
    Keeps Y and Z axes unchanged.
    """
    lines = mission_text.split('\n')
    mirrored_lines = []
    
    for line in lines:
        # Check if this line contains a movement command with [1, 0, 0] (X-axis)
        if '[1, 0, 0]' in line:
            # Find the number after [1, 0, 0], which is the distance
            match = re.search(r'\[1, 0, 0\],\s*([-+]?\d+\.?\d*)', line)
            if match:
                old_value = match.group(1)
                # Flip the sign
                new_value = str(-float(old_value))
                # Replace in the line
                new_line = line.replace(f'[1, 0, 0], {old_value}', f'[1, 0, 0], {new_value}')
                mirrored_lines.append(new_line)
            else:
                mirrored_lines.append(line)
        else:
            mirrored_lines.append(line)
    
    return '\n'.join(mirrored_lines)

def main():
    if len(sys.argv) != 3:
        print("Usage: python mirror_mission.py input_mission.txt output_mission.txt")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    
    # Read input file
    try:
        with open(input_file, 'r') as f:
            mission_text = f.read()
    except FileNotFoundError:
        print(f"Error: Input file '{input_file}' not found.")
        sys.exit(1)
    
    # Mirror the mission
    mirrored_mission = mirror_x_axis(mission_text)
    
    # Write output file
    with open(output_file, 'w') as f:
        f.write(mirrored_mission)
    
    print(f"Mirrored mission written to '{output_file}'")

if __name__ == "__main__":
    main()