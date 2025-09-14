#!/usr/bin/env python3
"""
Lightweight Robot Animation Visualizer
Memory-optimized version for GIF generation with reduced dataset size
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import os
import sys

class LightweightRobotAnimator:
    def __init__(self, data_path, max_frames=200):
        self.data_path = data_path
        self.max_frames = max_frames
        self.load_data()
        self.setup_visualization()
        
    def load_data(self):
        """Load and heavily subsample robot trajectory data"""
        # Load odometry data
        odom_file = os.path.join(self.data_path, 'maze', 'odom_data.csv')
        full_odom_data = pd.read_csv(odom_file)
        
        # Calculate subsample factor to get desired number of frames
        total_points = len(full_odom_data)
        self.subsample_factor = max(1, total_points // self.max_frames)
        
        self.odom_data = full_odom_data[::self.subsample_factor].reset_index(drop=True)
        
        # Limit to max_frames
        if len(self.odom_data) > self.max_frames:
            self.odom_data = self.odom_data[:self.max_frames]
        
        print(f"Reduced from {total_points} to {len(self.odom_data)} trajectory points")
        print(f"Subsample factor: {self.subsample_factor}")
        
    def setup_visualization(self):
        """Setup lightweight matplotlib figure"""
        # Smaller figure for memory efficiency
        self.fig, self.ax = plt.subplots(1, 1, figsize=(10, 8))
        
        self.ax.set_aspect('equal')
        self.ax.set_title('Robot Animation - Trajectory', fontsize=14, fontweight='bold')
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.grid(True, alpha=0.3)
        
        # Set axis limits with margin
        x_margin = (self.odom_data['x'].max() - self.odom_data['x'].min()) * 0.1
        y_margin = (self.odom_data['y'].max() - self.odom_data['y'].min()) * 0.1
        
        self.ax.set_xlim(self.odom_data['x'].min() - x_margin, 
                        self.odom_data['x'].max() + x_margin)
        self.ax.set_ylim(self.odom_data['y'].min() - y_margin, 
                        self.odom_data['y'].max() + y_margin)
        
        # Initialize plot elements
        self.trajectory_line, = self.ax.plot([], [], 'b-', linewidth=2, alpha=0.7, label='Trajectory')
        self.full_trajectory, = self.ax.plot(self.odom_data['x'], self.odom_data['y'], 
                                           'lightgray', linewidth=1, alpha=0.5, label='Full Path')
        self.robot_body = Circle((0, 0), 0.08, color='red', alpha=0.9)
        self.ax.add_patch(self.robot_body)
        
        # Add legend
        self.ax.legend(loc='upper right')
        
        # Add info text
        self.info_text = self.fig.text(0.02, 0.95, '', fontsize=10, verticalalignment='top',
                                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
    def animate_frame(self, frame):
        """Lightweight animation function"""
        if frame >= len(self.odom_data):
            return self.trajectory_line, self.robot_body
        
        # Get current robot state
        current_pose = self.odom_data.iloc[frame]
        x, y, theta = current_pose['x'], current_pose['y'], current_pose['theta']
        t = current_pose['t']
        
        # Update trajectory (only show recent path)
        trail_length = min(50, frame + 1)  # Show last 50 points
        start_idx = max(0, frame - trail_length + 1)
        traj_x = self.odom_data['x'][start_idx:frame+1]
        traj_y = self.odom_data['y'][start_idx:frame+1]
        self.trajectory_line.set_data(traj_x, traj_y)
        
        # Update robot position
        self.robot_body.center = (x, y)
        
        # Remove old arrow and create new one
        if hasattr(self, 'robot_arrow') and self.robot_arrow in self.ax.patches:
            self.robot_arrow.remove()
        
        # Direction arrow
        arrow_length = 0.12
        dx = arrow_length * np.cos(theta)
        dy = arrow_length * np.sin(theta)
        self.robot_arrow = self.ax.arrow(x, y, dx, dy, 
                                        head_width=0.025, head_length=0.025,
                                        fc='darkred', ec='darkred', alpha=0.9)
        
        # Update info text
        progress = (frame + 1) / len(self.odom_data) * 100
        info_str = f"Frame: {frame+1}/{len(self.odom_data)} ({progress:.1f}%)\n"
        info_str += f"Time: {t:.2f}s\n"
        info_str += f"Position: ({x:.3f}, {y:.3f})m\n"
        info_str += f"Orientation: {np.degrees(theta):.1f}°"
        
        self.info_text.set_text(info_str)
        
        return self.trajectory_line, self.robot_body
    
    def create_animation(self, save_gif=False, gif_filename='robot_animation_lite.gif'):
        """Create memory-efficient animation"""
        print(f"Creating lightweight animation with {len(self.odom_data)} frames...")
        
        # Create animation with optimized settings
        self.anim = animation.FuncAnimation(
            self.fig, self.animate_frame, frames=len(self.odom_data),
            interval=100, blit=True, repeat=True
        )
        
        if save_gif:
            print("Saving lightweight GIF...")
            # Very memory-efficient GIF settings - remove incompatible kwargs
            self.anim.save(gif_filename, writer='pillow', fps=8, dpi=60)
            print(f"Animation saved as {gif_filename}")
            return gif_filename
        else:
            plt.tight_layout()
            plt.show()
            return self.anim

def main():
    """Main function for lightweight robot animation"""
    # Set data path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_path = os.path.join(script_dir, 'data')
    
    if not os.path.exists(data_path):
        print(f"Error: Data directory not found at {data_path}")
        return
    
    print("="*60)
    print("Lightweight Robot Animation Visualizer")
    print("="*60)
    
    try:
        print("\nAnimation Options:")
        print("1. Quick preview (100 frames)")
        print("2. Medium quality (200 frames)")
        print("3. High quality (400 frames)")
        print("4. Just play in window")
        
        choice = input("Choose option (1-4): ").strip()
        
        if choice == '1':
            max_frames = 100
            save_gif = True
        elif choice == '2':
            max_frames = 200
            save_gif = True
        elif choice == '3':
            max_frames = 400
            save_gif = True
        else:
            max_frames = 200
            save_gif = False
        
        # Create animator
        animator = LightweightRobotAnimator(data_path, max_frames=max_frames)
        
        if save_gif:
            gif_name = f'robot_animation_{max_frames}frames.gif'
            result = animator.create_animation(save_gif=True, gif_filename=gif_name)
            print(f"\n✓ GIF created successfully: {result}")
            print(f"File size optimized for {max_frames} frames")
        else:
            print("\nStarting interactive animation...")
            print("Close window to exit")
            animator.create_animation(save_gif=False)
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
