#!/usr/bin/env python3
"""
Visualize car paths in 2D workspace with obstacles
"""
import numpy as np
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

def read_car_path(filename):
    """Read car path from file"""
    data = np.loadtxt(filename)
    # Columns: x, y, theta, v, control1, control2, duration
    x = data[:, 0]
    y = data[:, 1]
    theta = data[:, 2]
    v = data[:, 3]
    return x, y, theta, v

def draw_car_square(ax, x, y, theta, size=5.0, color='blue', alpha=0.3):
    """Draw a square representing the car at position (x, y) with orientation theta"""
    # Car is centered at (x, y) with side length size
    half_size = size / 2.0
    
    # Define corners of the square in car's local frame (centered at origin)
    corners_local = np.array([
        [-half_size, -half_size],
        [half_size, -half_size],
        [half_size, half_size],
        [-half_size, half_size],
        [-half_size, -half_size]  # Close the square
    ])
    
    # Rotation matrix
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    R = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
    
    # Rotate and translate corners
    corners_global = corners_local @ R.T + np.array([x, y])
    
    # Draw the square
    ax.fill(corners_global[:, 0], corners_global[:, 1], color=color, alpha=alpha, edgecolor=color, linewidth=1.5)
    
    # Draw heading direction
    dx = np.cos(theta) * half_size * 1.5
    dy = np.sin(theta) * half_size * 1.5
    ax.arrow(x, y, dx, dy, head_width=1.0, head_length=0.8, fc='red', ec='red', alpha=0.8, linewidth=2)

def plot_car_trajectory(x, y, theta, title="Car Trajectory"):
    """Plot the car trajectory with obstacles and car as square"""
    fig, ax = plt.subplots(figsize=(12, 12))
    
    # Draw obstacles FIRST (so they appear behind)
    for obs in obstacles:
        rect = RectPatch((obs['x'], obs['y']), obs['width'], obs['height'],
                        linewidth=2, edgecolor='black', facecolor='gray', alpha=0.7, zorder=1)
        ax.add_patch(rect)
    
    # Plot trajectory line
    ax.plot(x, y, 'b-', linewidth=2, alpha=0.5, label='Trajectory (center)', zorder=2)
    
    # Draw car squares at selected positions
    n_cars = min(12, len(x))
    car_indices = np.linspace(0, len(x)-1, n_cars, dtype=int)
    for i in car_indices:
        if i == 0:
            draw_car_square(ax, x[i], y[i], theta[i], size=5.0, color='green', alpha=0.4)
        elif i == car_indices[-1]:
            draw_car_square(ax, x[i], y[i], theta[i], size=5.0, color='red', alpha=0.4)
        else:
            draw_car_square(ax, x[i], y[i], theta[i], size=5.0, color='blue', alpha=0.25)
    
    # Mark start and goal centers
    ax.plot(x[0], y[0], 'go', markersize=12, label='Start', zorder=10)
    ax.plot(x[-1], y[-1], 'r*', markersize=18, label='Goal', zorder=10)
    
    # Labels and formatting
    ax.set_xlabel('X Position', fontsize=14)
    ax.set_ylabel('Y Position', fontsize=14)
    ax.set_title(title, fontsize=16, fontweight='bold')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=12, loc='best')
    ax.set_xlim(-55, 55)
    ax.set_ylim(-55, 55)
    
    plt.tight_layout()
    return fig

def plot_car_states(x, y, theta, v, title="Car States over Time"):
    """Plot car states over time"""
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 10))
    
    time = np.arange(len(x))
    
    # Plot x position
    ax1.plot(time, x, 'b-', linewidth=2)
    ax1.set_ylabel('X Position', fontsize=12)
    ax1.set_title('X Position over Time', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=-40, color='g', linestyle='--', alpha=0.5, label='Start')
    ax1.axhline(y=40, color='r', linestyle='--', alpha=0.5, label='Goal')
    ax1.legend()
    
    # Plot y position
    ax2.plot(time, y, 'g-', linewidth=2)
    ax2.set_ylabel('Y Position', fontsize=12)
    ax2.set_title('Y Position over Time', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=-40, color='g', linestyle='--', alpha=0.5, label='Start')
    ax2.axhline(y=40, color='r', linestyle='--', alpha=0.5, label='Goal')
    ax2.legend()
    
    # Plot theta (heading)
    ax3.plot(time, theta, 'r-', linewidth=2)
    ax3.set_xlabel('Time Step', fontsize=12)
    ax3.set_ylabel('θ (Heading) [rad]', fontsize=12)
    ax3.set_title('Heading Angle over Time', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    
    # Plot velocity
    ax4.plot(time, v, 'm-', linewidth=2)
    ax4.set_xlabel('Time Step', fontsize=12)
    ax4.set_ylabel('Velocity', fontsize=12)
    ax4.set_title('Velocity over Time', fontsize=12, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    
    fig.suptitle(title, fontsize=14, fontweight='bold', y=1.00)
    plt.tight_layout()
    return fig

def plot_car_workspace_detailed(x, y, theta):
    """Create a detailed workspace plot with car as squares"""
    fig, ax = plt.subplots(figsize=(14, 14))
    
    # Draw obstacles with labels
    for i, obs in enumerate(obstacles):
        rect = RectPatch((obs['x'], obs['y']), obs['width'], obs['height'],
                        linewidth=2, edgecolor='black', facecolor='gray', alpha=0.7, zorder=1)
        ax.add_patch(rect)
        # Add obstacle labels
        if i >= 4:  # Only label the middle obstacles
            cx = obs['x'] + obs['width']/2
            cy = obs['y'] + obs['height']/2
            ax.text(cx, cy, f'Obs {i-3}', ha='center', va='center', 
                   fontsize=10, fontweight='bold', color='white')
    
    # Plot trajectory with color gradient
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    
    from matplotlib.collections import LineCollection
    colors = plt.cm.viridis(np.linspace(0, 1, len(x)))
    lc = LineCollection(segments, colors=colors, linewidth=2, alpha=0.6, zorder=2)
    ax.add_collection(lc)
    
    # Add colorbar
    sm = plt.cm.ScalarMappable(cmap=plt.cm.viridis, 
                               norm=plt.Normalize(vmin=0, vmax=len(x)-1))
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, pad=0.02)
    cbar.set_label('Time Step', fontsize=12)
    
    # Draw car as 5x5 squares at key positions
    n_cars = 10
    car_indices = np.linspace(0, len(x)-1, n_cars, dtype=int)
    for idx, i in enumerate(car_indices):
        if i == 0:
            draw_car_square(ax, x[i], y[i], theta[i], size=5.0, color='green', alpha=0.5)
        elif i == car_indices[-1]:
            draw_car_square(ax, x[i], y[i], theta[i], size=5.0, color='red', alpha=0.5)
        else:
            alpha_val = 0.2 + (idx / len(car_indices)) * 0.3
            draw_car_square(ax, x[i], y[i], theta[i], size=5.0, color='blue', alpha=alpha_val)
    
    # Mark start and goal centers
    ax.plot(x[0], y[0], 'go', markersize=15, label='Start', zorder=10, 
           markeredgecolor='darkgreen', markeredgewidth=2)
    ax.plot(x[-1], y[-1], 'r*', markersize=20, label='Goal', zorder=10,
           markeredgecolor='darkred', markeredgewidth=2)
    
    # Labels and formatting
    ax.set_xlabel('X Position', fontsize=14, fontweight='bold')
    ax.set_ylabel('Y Position', fontsize=14, fontweight='bold')
    ax.set_title('Car Navigation (5×5 Square) in Street Environment', fontsize=16, fontweight='bold')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.legend(fontsize=12, loc='upper right')
    ax.set_xlim(-55, 55)
    ax.set_ylim(-55, 55)
    
    # Add boundary labels
    ax.text(0, -52, 'Street Environment with Obstacles', ha='center', fontsize=12, 
           style='italic', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    return fig

if __name__ == "__main__":
    # Read the path
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "car_path.txt"
    
    try:
        x, y, theta, v = read_car_path(filename)
        
        # Determine planner from filename
        if "rrt" in filename.lower():
            planner = "RRT"
        elif "kpiece" in filename.lower():
            planner = "KPIECE1"
        else:
            planner = "Unknown"
        
        # Create trajectory plot
        fig1 = plot_car_trajectory(x, y, theta, f"Car Trajectory - {planner}")
        fig1.savefig(filename.replace('.txt', '_trajectory.png'), dpi=150, bbox_inches='tight')
        print(f"Saved: {filename.replace('.txt', '_trajectory.png')}")
        
        # Create detailed workspace plot
        fig2 = plot_car_workspace_detailed(x, y, theta)
        fig2.savefig(filename.replace('.txt', '_workspace.png'), dpi=150, bbox_inches='tight')
        print(f"Saved: {filename.replace('.txt', '_workspace.png')}")
        
        # Create states plot
        fig3 = plot_car_states(x, y, theta, v, f"Car States - {planner}")
        fig3.savefig(filename.replace('.txt', '_states.png'), dpi=150, bbox_inches='tight')
        print(f"Saved: {filename.replace('.txt', '_states.png')}")
        
        plt.show()
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
