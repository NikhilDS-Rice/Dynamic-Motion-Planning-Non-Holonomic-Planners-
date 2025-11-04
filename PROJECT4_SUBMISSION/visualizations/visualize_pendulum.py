#!/usr/bin/env python3
"""
Visualize pendulum paths in phase space (theta vs omega)
"""
import numpy as np
import matplotlib.pyplot as plt
import sys

def read_path(filename):
    """Read path from file"""
    data = np.loadtxt(filename)
    # Columns: theta, omega, control, duration
    theta = data[:, 0]
    omega = data[:, 1]
    return theta, omega

def plot_pendulum_phase_space(theta, omega, title="Pendulum Phase Space"):
    """Plot the pendulum trajectory in phase space"""
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Plot trajectory
    ax.plot(theta, omega, 'b-', linewidth=2, alpha=0.7, label='Trajectory')
    
    # Mark start and goal
    ax.plot(theta[0], omega[0], 'go', markersize=15, label='Start', zorder=5)
    ax.plot(theta[-1], omega[-1], 'r*', markersize=20, label='Goal', zorder=5)
    
    # Add arrows to show direction
    n_arrows = 10
    arrow_indices = np.linspace(0, len(theta)-2, n_arrows, dtype=int)
    for i in arrow_indices:
        dx = theta[i+1] - theta[i]
        dy = omega[i+1] - omega[i]
        ax.arrow(theta[i], omega[i], dx*0.3, dy*0.3, 
                head_width=0.15, head_length=0.1, fc='blue', ec='blue', alpha=0.5)
    
    # Labels and formatting
    ax.set_xlabel('θ (Angle) [radians]', fontsize=14)
    ax.set_ylabel('ω (Angular Velocity) [rad/s]', fontsize=14)
    ax.set_title(title, fontsize=16, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=12)
    
    # Add reference lines
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax.axvline(x=0, color='k', linestyle='--', alpha=0.3)
    ax.axvline(x=-np.pi/2, color='g', linestyle='--', alpha=0.3, label='Start (hanging)')
    ax.axvline(x=np.pi/2, color='r', linestyle='--', alpha=0.3, label='Goal (up)')
    
    plt.tight_layout()
    return fig

def plot_pendulum_time_series(theta, omega, title="Pendulum States over Time"):
    """Plot theta and omega over time"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    time = np.arange(len(theta))
    
    # Plot theta
    ax1.plot(time, theta, 'b-', linewidth=2)
    ax1.axhline(y=-np.pi/2, color='g', linestyle='--', alpha=0.5, label='Start')
    ax1.axhline(y=np.pi/2, color='r', linestyle='--', alpha=0.5, label='Goal')
    ax1.set_ylabel('θ (Angle) [rad]', fontsize=12)
    ax1.set_title(title, fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Plot omega
    ax2.plot(time, omega, 'r-', linewidth=2)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax2.set_xlabel('Time Step', fontsize=12)
    ax2.set_ylabel('ω (Angular Velocity) [rad/s]', fontsize=12)
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

if __name__ == "__main__":
    # Read the path
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "pendulum_path.txt"
    
    try:
        theta, omega = read_path(filename)
        
        # Determine planner from filename or use default
        if "rrt" in filename.lower():
            planner = "RRT"
        elif "kpiece" in filename.lower():
            planner = "KPIECE1"
        else:
            planner = "Unknown"
        
        # Create phase space plot
        fig1 = plot_pendulum_phase_space(theta, omega, 
                                        f"Pendulum Phase Space - {planner}")
        fig1.savefig(filename.replace('.txt', '_phase.png'), dpi=150, bbox_inches='tight')
        print(f"Saved: {filename.replace('.txt', '_phase.png')}")
        
        # Create time series plot
        fig2 = plot_pendulum_time_series(theta, omega,
                                        f"Pendulum States over Time - {planner}")
        fig2.savefig(filename.replace('.txt', '_time.png'), dpi=150, bbox_inches='tight')
        print(f"Saved: {filename.replace('.txt', '_time.png')}")
        
        plt.show()
        
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
