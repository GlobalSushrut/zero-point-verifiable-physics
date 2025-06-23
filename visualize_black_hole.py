#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import matplotlib.colors as mcolors
import os
import sys

# File to read trajectory data
trajectory_file = 'black_hole_trajectories.dat'

def read_trajectory_data(filename):
    """Read trajectory data from file"""
    if not os.path.exists(filename):
        print(f"Error: File {filename} not found!")
        sys.exit(1)
        
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

def create_static_plot(trajectories, output_file='black_hole_visualization.png'):
    """Create a static 3D plot of trajectories"""
    # Create figure
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Choose a subset of trajectories for clarity
    trajectory_subset = {}
    for i, (traj_id, points) in enumerate(trajectories.items()):
        if i % 10 == 0:  # Only plot every 10th trajectory
            trajectory_subset[traj_id] = points
    
    # Get colormap
    cmap = plt.get_cmap('viridis')
    colors = [cmap(i/len(trajectory_subset)) for i in range(len(trajectory_subset))]
    
    # Plot black hole as a sphere at origin
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    r = 1.0  # Schwarzschild radius
    x = r * np.outer(np.cos(u), np.sin(v))
    y = r * np.outer(np.sin(u), np.sin(v))
    z = r * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, y, z, color='black', alpha=0.7)
    
    # Plot trajectories
    for i, (traj_id, points) in enumerate(trajectory_subset.items()):
        color = colors[i % len(colors)]
        ax.plot(points[:, 0], points[:, 1], points[:, 2], 
                 '-', linewidth=1.5, alpha=0.7, 
                 color=color, label=f'Particle {traj_id}')
        
        # Mark starting point with a dot
        ax.scatter(points[0, 0], points[0, 1], points[0, 2], 
                   marker='o', s=30, color=color)
    
    # Set axis limits
    max_range = 10
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(-max_range, max_range)
    
    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Black Hole Simulation Trajectories', fontsize=16)
    
    # Add text annotation for the event horizon
    ax.text(0, 0, -2.5, "Black Hole\n(Event Horizon)", 
            color='white', fontsize=12, 
            horizontalalignment='center')
    
    # Add grid
    ax.grid(True, alpha=0.3)
    
    # Optimize viewing angle
    ax.view_init(elev=30, azim=45)
    
    # Save figure
    plt.tight_layout()
    plt.savefig(output_file, dpi=300)
    print(f"Static visualization saved as {output_file}")
    
    # Show plot
    plt.show()

def create_animation(trajectories, output_file='black_hole_animation.gif'):
    """Create an animated visualization of trajectories"""
    try:
        import imageio
    except ImportError:
        print("Animation requires imageio module. Install with: pip install imageio")
        return
        
    # Choose subset of trajectories
    trajectory_subset = {}
    for i, (traj_id, points) in enumerate(trajectories.items()):
        if i % 20 == 0:  # Only animate every 20th trajectory
            trajectory_subset[traj_id] = points
    
    # Get max number of points in any trajectory
    max_points = max([len(points) for points in trajectory_subset.values()])
    
    # Determine number of frames for animation
    frames = min(100, max_points)
    frame_step = max(1, max_points // frames)
    
    # Get colormap
    cmap = plt.get_cmap('viridis')
    colors = [cmap(i/len(trajectory_subset)) for i in range(len(trajectory_subset))]
    
    # Create frames
    print("Creating animation frames...")
    filenames = []
    for frame in range(0, max_points, frame_step):
        # Create figure
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot black hole
        u = np.linspace(0, 2 * np.pi, 50)
        v = np.linspace(0, np.pi, 50)
        r = 1.0  # Schwarzschild radius
        x = r * np.outer(np.cos(u), np.sin(v))
        y = r * np.outer(np.sin(u), np.sin(v))
        z = r * np.outer(np.ones(np.size(u)), np.cos(v))
        ax.plot_surface(x, y, z, color='black', alpha=0.7)
        
        # Plot trajectories up to current frame
        for i, (traj_id, points) in enumerate(trajectory_subset.items()):
            current_frame = min(frame, len(points)-1)
            color = colors[i % len(colors)]
            
            # Plot trajectory up to current point
            ax.plot(points[:current_frame+1, 0], 
                    points[:current_frame+1, 1], 
                    points[:current_frame+1, 2],
                    '-', linewidth=1.5, color=color, alpha=0.7)
            
            # Plot current position as a dot
            if current_frame < len(points):
                ax.scatter(points[current_frame, 0], 
                          points[current_frame, 1], 
                          points[current_frame, 2],
                          marker='o', s=30, color=color)
        
        # Set axis limits
        max_range = 10
        ax.set_xlim(-max_range, max_range)
        ax.set_ylim(-max_range, max_range)
        ax.set_zlim(-max_range, max_range)
        
        # Set labels and title
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'Black Hole Simulation - Frame {frame/frame_step}/{frames-1}', 
                     fontsize=16)
        
        # Optimize viewing angle
        ax.view_init(elev=30, azim=45 + (frame/frame_step) * 0.5)  # Rotate view slightly
        
        # Save frame
        frame_file = f'frame_{frame:04d}.png'
        plt.savefig(frame_file, dpi=150)
        filenames.append(frame_file)
        
        plt.close()
        
        # Show progress
        if frame % (5 * frame_step) == 0:
            print(f"Created frame {frame/frame_step}/{frames-1}")
    
    # Create GIF animation
    print("Creating GIF animation...")
    with imageio.get_writer(output_file, mode='I', duration=0.1) as writer:
        for filename in filenames:
            image = imageio.imread(filename)
            writer.append_data(image)
            
    # Clean up frame files
    print("Cleaning up frame files...")
    for filename in filenames:
        os.remove(filename)
        
    print(f"Animation saved as {output_file}")

def create_top_view_plot(trajectories, output_file='black_hole_top_view.png'):
    """Create a 2D top view (x-y plane) plot of trajectories"""
    plt.figure(figsize=(12, 10))
    
    # Plot black hole as a circle at origin
    circle = plt.Circle((0, 0), 1.0, color='black', alpha=0.7)
    plt.gca().add_patch(circle)
    
    # Choose a subset of trajectories for clarity
    trajectory_subset = {}
    for i, (traj_id, points) in enumerate(trajectories.items()):
        if i % 5 == 0:  # Plot every 5th trajectory
            trajectory_subset[traj_id] = points
    
    # Get colormap
    cmap = plt.get_cmap('plasma')
    colors = [cmap(i/len(trajectory_subset)) for i in range(len(trajectory_subset))]
    
    # Plot trajectories
    for i, (traj_id, points) in enumerate(trajectory_subset.items()):
        color = colors[i % len(colors)]
        plt.plot(points[:, 0], points[:, 1], '-', linewidth=1.5, alpha=0.7, color=color)
        plt.scatter(points[0, 0], points[0, 1], marker='o', s=30, color=color)
    
    # Set axis limits
    max_range = 10
    plt.xlim(-max_range, max_range)
    plt.ylim(-max_range, max_range)
    
    # Set labels and title
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Black Hole Simulation Trajectories (Top View)', fontsize=16)
    
    # Add text annotation for the event horizon
    plt.text(0, 0, "Black Hole\n(Event Horizon)", 
             color='white', fontsize=12, 
             horizontalalignment='center')
    
    # Make axes equal to maintain proper aspect ratio
    plt.axis('equal')
    
    # Add grid
    plt.grid(True, alpha=0.3)
    
    # Save figure
    plt.tight_layout()
    plt.savefig(output_file, dpi=300)
    print(f"Top view visualization saved as {output_file}")
    
    # Show plot
    plt.show()

if __name__ == "__main__":
    print("Reading trajectory data...")
    trajectories = read_trajectory_data(trajectory_file)
    
    print(f"Loaded {len(trajectories)} particle trajectories")
    
    # Create plots
    create_static_plot(trajectories)
    create_top_view_plot(trajectories)
    
    # Animation is optional as it takes more time
    animation_choice = input("Create animation? (y/n, may take a few minutes): ")
    if animation_choice.lower() == 'y':
        try:
            create_animation(trajectories)
        except Exception as e:
            print(f"Error creating animation: {e}")
            print("Try installing required packages with: pip install matplotlib numpy imageio")
