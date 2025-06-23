#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# File to read trajectory data
trajectory_file = 'black_hole_trajectories.dat'

def read_trajectory_data(filename):
    """Read trajectory data from file"""
    if not os.path.exists(filename):
        print(f"Error: File {filename} not found!")
        return {}
        
    trajectories = {}
    
    with open(filename, 'r') as file:
        for line in file:
            # Skip comment lines
            if line.startswith('#'):
                continue
            
            # Skip empty lines
            if not line.strip():
                continue
                
            # Parse data
            parts = line.strip().split()
            if len(parts) >= 4:  # At least id, x, y, z
                traj_id = int(parts[0])
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                
                # Initialize trajectory if not exists
                if traj_id not in trajectories:
                    trajectories[traj_id] = []
                    
                # Add point to trajectory
                trajectories[traj_id].append((x, y, z))
    
    # Convert lists to numpy arrays
    for traj_id in trajectories:
        trajectories[traj_id] = np.array(trajectories[traj_id])
        
    return trajectories

def create_2d_plots(trajectories):
    """Create 2D plots for top, side, and front views"""
    # Create figure with subplots
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    
    # Choose a subset of trajectories for clarity
    trajectory_subset = {}
    for i, (traj_id, points) in enumerate(trajectories.items()):
        if i % 10 == 0:  # Only plot every 10th trajectory
            trajectory_subset[traj_id] = points
    
    # Views: XY (top), XZ (front), YZ (side)
    views = [(0, 1, 'XY Plane (Top View)'), 
             (0, 2, 'XZ Plane (Front View)'), 
             (1, 2, 'YZ Plane (Side View)')]
    
    # Plot each view
    for i, (dim1, dim2, title) in enumerate(views):
        ax = axes[i]
        
        # Plot event horizon as a circle
        if dim1 == 0 and dim2 == 1:  # Only for top view
            circle = plt.Circle((0, 0), 1.0, color='black', alpha=0.7)
            ax.add_patch(circle)
        else:
            # For other views we'll plot a line
            ax.plot([-1, 1], [0, 0], 'k-', lw=2, alpha=0.7)
        
        # Plot trajectories
        for traj_id, points in trajectory_subset.items():
            # Use different color for each trajectory
            color = plt.cm.viridis(traj_id / len(trajectories))
            
            # Plot trajectory
            ax.plot(points[:, dim1], points[:, dim2], '-', linewidth=1, alpha=0.6, color=color)
            
            # Mark start point
            ax.scatter(points[0, dim1], points[0, dim2], s=20, color=color)
        
        # Set axis labels
        ax.set_xlabel('XYZ'[dim1])
        ax.set_ylabel('XYZ'[dim2])
        
        # Set title
        ax.set_title(title)
        
        # Set axis limits
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        
        # Add grid
        ax.grid(True, alpha=0.3)
    
    # Add overall title
    plt.suptitle('Black Hole Simulation Trajectories', fontsize=16)
    
    # Adjust layout
    plt.tight_layout()
    
    # Save figure
    plt.savefig('black_hole_2d_views.png', dpi=200)
    print("2D views saved as black_hole_2d_views.png")
    
    # Show plot
    plt.show()

def create_3d_plot(trajectories):
    """Create a 3D plot of trajectories"""
    # Create figure
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Choose a subset of trajectories for clarity
    trajectory_subset = {}
    for i, (traj_id, points) in enumerate(trajectories.items()):
        if i % 20 == 0:  # Only plot every 20th trajectory
            trajectory_subset[traj_id] = points
    
    # Plot black hole as a wireframe sphere at origin
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 20)
    r = 1.0  # Schwarzschild radius
    x = r * np.outer(np.cos(u), np.sin(v))
    y = r * np.outer(np.sin(u), np.sin(v))
    z = r * np.outer(np.ones(np.size(u)), np.cos(v))
    
    ax.plot_wireframe(x, y, z, color='black', alpha=0.5, linewidth=0.5)
    
    # Plot trajectories
    for traj_id, points in trajectory_subset.items():
        # Use different color for each trajectory
        color = plt.cm.plasma(traj_id / len(trajectories))
        
        # Plot trajectory
        ax.plot(points[:, 0], points[:, 1], points[:, 2], 
                '-', linewidth=1, alpha=0.7, color=color)
        
        # Mark start point
        ax.scatter(points[0, 0], points[0, 1], points[0, 2], 
                  s=20, color=color)
    
    # Set axis limits
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-10, 10)
    
    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    # Set title
    ax.set_title('Black Hole Simulation - 3D Trajectories')
    
    # Set optimized viewing angle
    ax.view_init(elev=30, azim=45)
    
    # Save figure
    plt.savefig('black_hole_3d_view.png', dpi=200)
    print("3D view saved as black_hole_3d_view.png")
    
    # Show plot
    plt.show()

if __name__ == "__main__":
    print("Reading trajectory data from", trajectory_file)
    trajectories = read_trajectory_data(trajectory_file)
    
    if not trajectories:
        print("No trajectory data found!")
    else:
        print(f"Loaded {len(trajectories)} particle trajectories")
        
        print("Creating 2D plots...")
        create_2d_plots(trajectories)
        
        print("Creating 3D plot...")
        create_3d_plot(trajectories)
