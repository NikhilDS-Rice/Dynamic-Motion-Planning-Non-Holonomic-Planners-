#!/usr/bin/env python3
"""
Plot the car environment with obstacles, start, and goal
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle as RectPatch
import sys

# Define obstacles (matching the C++ code)
obstacles = [
    # Left wall
    {'x': -50, 'y': -50, 'width': 5, 'height': 100},
    # Right wall
    {'x': 45, 'y': -50, 'width': 5, 'height': 100},
    # Top wall
    {'x': -50, 'y': 45, 'width': 100, 'height': 5},
    # Bottom wall
    {'x': -50, 'y': -50, 'width': 100, 'height': 5},
    # Obstacle 5
    {'x': -25, 'y': -45, 'width': 10, 'height': 40},
    # Obstacle 6
    {'x': -25, 'y': 5, 'width': 10, 'height': 40},
    # Obstacle 7
    {'x': -5, 'y': -25, 'width': 10, 'height': 70},
    # Obstacle 8
    {'x': 25, 'y': -5, 'width': 20, 'height': 10}
]

# Start and goal positions
start_x, start_y = -40.0, -40.0
goal_x, goal_y = 40.0, 40.0

def plot_environment(output_file='car_environment.png'):
    """Plot the car environment"""
    fig, ax = plt.subplots(figsize=(12, 12))
    
    # Draw obstacles
    for i, obs in enumerate(obstacles):
        rect = RectPatch((obs['x'], obs['y']), obs['width'], obs['height'],
                        linewidth=2, edgecolor='black', facecolor='gray', alpha=0.7, zorder=1)
        ax.add_patch(rect)
        
        # Label interior obstacles
        if i >= 4:
            cx = obs['x'] + obs['width']/2
            cy = obs['y'] + obs['height']/2
            ax.text(cx, cy, f'Obs {i-3}', ha='center', va='center', 
                   fontsize=12, fontweight='bold', color='white')
    
    # Mark start and goal
    ax.plot(start_x, start_y, 'go', markersize=20, label='Start (-40, -40)', 
            zorder=5, markeredgecolor='darkgreen', markeredgewidth=2)
    ax.plot(goal_x, goal_y, 'r*', markersize=25, label='Goal (40, 40)', 
            zorder=5, markeredgecolor='darkred', markeredgewidth=2)
    
    # Add 5x5 car footprint at start for reference
    car_size = 5.0
    half_car = car_size / 2.0
    car_rect = RectPatch((start_x - half_car, start_y - half_car), car_size, car_size,
                         linewidth=2, edgecolor='green', facecolor='lightgreen', 
                         alpha=0.5, zorder=4, linestyle='--', label='Car (5×5)')
    ax.add_patch(car_rect)
    
    # Labels and formatting
    ax.set_xlabel('X Position', fontsize=14, fontweight='bold')
    ax.set_ylabel('Y Position', fontsize=14, fontweight='bold')
    ax.set_title('Car Environment with Obstacles', fontsize=16, fontweight='bold')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.legend(fontsize=12, loc='upper right')
    ax.set_xlim(-55, 55)
    ax.set_ylim(-55, 55)
    
    # Add annotations
    ax.text(0, -52, 'Workspace: 100×100 units | Car: 5×5 square', 
           ha='center', fontsize=11, style='italic',
           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f'Saved: {output_file}')
    plt.close()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        output_file = sys.argv[1]
    else:
        output_file = 'car_environment.png'
    
    plot_environment(output_file)
