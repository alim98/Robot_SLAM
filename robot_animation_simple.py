#!/usr/bin/env python3
"""
Simple Robot Animation - Fixed version
Creates a basic animated GIF of robot trajectory without complex features
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import os

class SimpleRobotAnimator:
    def __init__(self, data_path, max_frames=150):
        self.data_path = data_path
        self.max_frames = max_frames
        self.load_data()
        self.setup_plot()
        
    def load_data(self):
        """Load and subsample trajectory data"""
        odom_file = os.path.join(self.data_path, 'maze', 'odom_data.csv')
        full_data = pd.read_csv(odom_file)
        
        # Calculate subsample to get desired frames
        step = max(1, len(full_data) // self.max_frames)
        self.data = full_data[::step][:self.max_frames].reset_index(drop=True)
        
        print(f"Using {len(self.data)} frames from {len(full_data)} total points")
        
    def setup_plot(self):
        """Setup matplotlib figure"""
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_aspect('equal')
        self.ax.set_title('Robot Trajectory Animation', fontsize=14)
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.grid(True, alpha=0.3)
        
        # Set limits
        margin = 0.5
        self.ax.set_xlim(self.data['x'].min() - margin, self.data['x'].max() + margin)
        self.ax.set_ylim(self.data['y'].min() - margin, self.data['y'].max() + margin)
        
        # Plot full path in light gray
        self.ax.plot(self.data['x'], self.data['y'], 'lightgray', alpha=0.5, linewidth=1)
        
        # Initialize animated elements
        self.trail_line, = self.ax.plot([], [], 'b-', linewidth=2, label='Trail')
        self.robot = Circle((0, 0), 0.1, color='red', alpha=0.8)
        self.ax.add_patch(self.robot)
        
        self.ax.legend()
        
    def animate(self, frame):
        """Animation function"""
        if frame >= len(self.data):
            return self.trail_line, self.robot
            
        # Current position
        x = self.data.iloc[frame]['x']
        y = self.data.iloc[frame]['y']
        
        # Update robot position
        self.robot.center = (x, y)
        
        # Update trail (show last 30 points)
        start_idx = max(0, frame - 29)
        trail_x = self.data['x'][start_idx:frame+1]
        trail_y = self.data['y'][start_idx:frame+1]
        self.trail_line.set_data(trail_x, trail_y)
        
        return self.trail_line, self.robot
    
    def create_gif(self, filename='robot_simple.gif'):
        """Create and save GIF"""
        print("Creating animation...")
        
        anim = animation.FuncAnimation(
            self.fig, self.animate, frames=len(self.data),
            interval=150, blit=True, repeat=True
        )
        
        print(f"Saving GIF: {filename}")
        anim.save(filename, writer='pillow', fps=6)
        print(f"✓ GIF saved successfully: {filename}")
        
        plt.close()
        return filename

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_path = os.path.join(script_dir, 'data')
    
    if not os.path.exists(data_path):
        print("Error: Data directory not found")
        return
    
    print("Simple Robot Animation Generator")
    print("=" * 40)
    
    try:
        animator = SimpleRobotAnimator(data_path, max_frames=150)
        gif_file = animator.create_gif()
        print(f"Animation complete! File: {gif_file}")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
