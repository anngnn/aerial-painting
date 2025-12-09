import matplotlib.pyplot as plt
import numpy as np

def plot_mission_xy(mission, start_pos=(0, 0)):
    """
    Plot the XY plane trajectory of the mission (rotated 180 degrees)
    
    Args:
        mission: list of mission waypoints
        start_pos: starting (x, y) position
    """
    # Parse mission and track positions
    positions = [start_pos]
    paint_dots = []
    
    current_x, current_y, current_z = start_pos[0], start_pos[1], 0.35  # Assume starting at z=0.35
    
    for step in mission:
        direction = step[0]
        distance = step[1]
        
        # Update position based on direction
        dx = direction[0] * distance
        dy = direction[1] * distance
        dz = direction[2] * distance
        
        current_x += dx
        current_y += dy
        current_z += dz
        
        # Only track XY movements
        if direction[2] == 0:  # Horizontal movement
            positions.append((current_x, current_y))
        elif direction[2] == -1:  # Going down (painting)
            # Mark this as a paint location (lowest Z point before going up)
            paint_dots.append((current_x, current_y))
    
    # Convert to arrays for plotting
    positions = np.array(positions)
    paint_dots = np.array(paint_dots)
    
    # Rotate 180 degrees: (x, y) -> (-x, -y)
    positions_rotated = np.column_stack([-positions[:, 0], -positions[:, 1]])
    if len(paint_dots) > 0:
        paint_dots_rotated = np.column_stack([-paint_dots[:, 0], -paint_dots[:, 1]])
    start_pos_rotated = (-start_pos[0], -start_pos[1])
    
    # Create plot
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Plot the path
    ax.plot(positions_rotated[:, 0], positions_rotated[:, 1], 'b-', linewidth=1, alpha=0.5, label='Flight path')
    
    # Plot waypoints
    ax.plot(positions_rotated[:, 0], positions_rotated[:, 1], 'bo', markersize=4, alpha=0.3, label='Waypoints')
    
    # Plot paint dots
    if len(paint_dots) > 0:
        ax.scatter(paint_dots_rotated[:, 0], paint_dots_rotated[:, 1], 
                  c='red', s=100, marker='o', 
                  edgecolors='darkred', linewidths=2,
                  label='Paint dots', zorder=5)
        
        # Number the dots
        for i, (x, y) in enumerate(paint_dots_rotated):
            ax.annotate(str(i+1), (x, y), 
                       fontsize=8, ha='center', va='center',
                       color='white', weight='bold', zorder=6)
    
    # Plot start position
    ax.scatter(start_pos_rotated[0], start_pos_rotated[1], 
              c='green', s=300, marker='*',
              edgecolors='black', linewidths=2,
              label='Start', zorder=5)
    
    # Add grid
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title('Drone Mission - XY Plane View (Rotated 180°)')
    ax.legend()
    ax.set_aspect('equal')
    
    # Add boundary box
    ax.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.3)
    ax.axvline(x=0, color='k', linestyle='--', linewidth=0.5, alpha=0.3)
    
    plt.tight_layout()
    return fig, ax


def print_mission_stats(mission):
    """Print statistics about the mission"""
    num_dots = sum(1 for step in mission if step[0] == [0, 0, -1] and len(step) == 4)
    
    # Calculate total distances
    total_distance = 0
    xy_distance = 0
    z_distance = 0
    
    for step in mission:
        distance = step[1]
        total_distance += distance
        
        if step[0][2] == 0:  # XY movement
            xy_distance += distance
        else:  # Z movement
            z_distance += distance
    
    print("\n" + "="*50)
    print("MISSION STATISTICS")
    print("="*50)
    print(f"Total dots to paint: {num_dots}")
    print(f"Total waypoints: {len(mission)}")
    print(f"Total distance: {total_distance:.2f} m")
    print(f"  - XY distance: {xy_distance:.2f} m")
    print(f"  - Z distance: {z_distance:.2f} m")
    print("="*50 + "\n")


# Load mission from file
with open('mission_fish.txt', 'r') as f:
    exec(f.read())

# Print stats
print_mission_stats(mission)

# Plot
fig, ax = plot_mission_xy(mission, start_pos=(0, 0))
plt.savefig('mission_xy_plot_rotated180.png', dpi=150, bbox_inches='tight')
print("Plot saved to 'mission_xy_plot_rotated180.png'")
plt.show()