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
    # Middle obstacle
    {'x': -10, 'y': -10, 'width': 20, 'height': 20},
    # Additional obstacle
    {'x': 15, 'y': -30, 'width': 15, 'height': 10}
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

def plot_car_trajectory(x, y, theta, title="Car Trajectory"):
    """Plot the car trajectory with obstacles"""
    fig, ax = plt.subplots(figsize=(12, 12))
    
    # Draw obstacles
    for obs in obstacles:
        rect = RectPatch((obs['x'], obs['y']), obs['width'], obs['height'],
                        linewidth=2, edgecolor='black', facecolor='gray', alpha=0.5)
        ax.add_patch(rect)
    
    # Plot trajectory
    ax.plot(x, y, 'b-', linewidth=2.5, alpha=0.7, label='Trajectory')
    
    # Mark start and goal
    ax.plot(x[0], y[0], 'go', markersize=15, label='Start', zorder=5)
    ax.plot(x[-1], y[-1], 'r*', markersize=20, label='Goal', zorder=5)
    
    # Add arrows to show direction and heading
    n_arrows = min(15, len(x)//3)
    arrow_indices = np.linspace(0, len(x)-1, n_arrows, dtype=int)
    for i in arrow_indices[:-1]:  # Skip last one
        # Direction arrow
        dx = np.cos(theta[i]) * 2
        dy = np.sin(theta[i]) * 2
        ax.arrow(x[i], y[i], dx, dy,
                head_width=1.5, head_length=1.0, fc='red', ec='red', alpha=0.6)
    
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
    """Create a detailed workspace plot"""
    fig, ax = plt.subplots(figsize=(14, 14))
    
    # Draw obstacles with labels
    for i, obs in enumerate(obstacles):
        rect = RectPatch((obs['x'], obs['y']), obs['width'], obs['height'],
                        linewidth=2, edgecolor='black', facecolor='gray', alpha=0.6)
        ax.add_patch(rect)
        # Add obstacle labels
        if i >= 4:  # Only label the middle obstacles
            cx = obs['x'] + obs['width']/2
            cy = obs['y'] + obs['height']/2
            ax.text(cx, cy, f'Obs {i-3}', ha='center', va='center', 
                   fontsize=10, fontweight='bold')
    
    # Plot trajectory with color gradient
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    
    from matplotlib.collections import LineCollection
    colors = plt.cm.viridis(np.linspace(0, 1, len(x)))
    lc = LineCollection(segments, colors=colors, linewidth=3, alpha=0.8)
    ax.add_collection(lc)
    
    # Add colorbar
    sm = plt.cm.ScalarMappable(cmap=plt.cm.viridis, 
                               norm=plt.Normalize(vmin=0, vmax=len(x)-1))
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, pad=0.02)
    cbar.set_label('Time Step', fontsize=12)
    
    # Mark start and goal with larger markers
    ax.plot(x[0], y[0], 'go', markersize=20, label='Start', zorder=10, 
           markeredgecolor='darkgreen', markeredgewidth=2)
    ax.plot(x[-1], y[-1], 'r*', markersize=25, label='Goal', zorder=10,
           markeredgecolor='darkred', markeredgewidth=2)
    
    # Add car orientation at key points
    n_cars = 8
    car_indices = np.linspace(0, len(x)-1, n_cars, dtype=int)
    car_length = 3
    for i in car_indices:
        # Draw car as a small rectangle with orientation
        dx = np.cos(theta[i]) * car_length
        dy = np.sin(theta[i]) * car_length
        ax.arrow(x[i], y[i], dx, dy,
                head_width=1.5, head_length=1.0, fc='blue', ec='blue', 
                alpha=0.7, linewidth=2)
    
    # Labels and formatting
    ax.set_xlabel('X Position', fontsize=14, fontweight='bold')
    ax.set_ylabel('Y Position', fontsize=14, fontweight='bold')
    ax.set_title('Car Navigation in Street Environment', fontsize=16, fontweight='bold')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.legend(fontsize=12, loc='upper right')
    ax.set_xlim(-55, 55)
    ax.set_ylim(-55, 55)
    
    # Add boundary labels
    ax.text(0, -52, 'Street Environment', ha='center', fontsize=12, 
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
