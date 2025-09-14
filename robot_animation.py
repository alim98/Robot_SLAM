#!/usr/bin/env python3
"""
Standalone Robot Animation Visualizer
Creates an animated visualization of the robot moving through the maze
with real-time sensor data, trajectory tracking, and interactive controls.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Polygon, Rectangle
from matplotlib.collections import LineCollection
import os
import sys

class RobotAnimator:
    def __init__(self, data_path):
        self.data_path = data_path
        self.load_data()
        self.setup_visualization()
        
    def load_data(self):
        """Load robot trajectory and sensor data"""
        # Load odometry data
        odom_file = os.path.join(self.data_path, 'maze', 'odom_data.csv')
        self.odom_data = pd.read_csv(odom_file)
        
        # Load range data
        range_file = os.path.join(self.data_path, 'maze', 'range_data.csv')
        self.range_data = pd.read_csv(range_file)
        
        # Subsample for smoother animation (more aggressive for GIF)
        self.subsample_factor = 50  # Reduced from 10 to 50 for memory efficiency
        self.odom_data = self.odom_data[::self.subsample_factor].reset_index(drop=True)
        self.range_data = self.range_data[::self.subsample_factor].reset_index(drop=True)
        
        print(f"Loaded {len(self.odom_data)} trajectory points")
        print(f"Loaded {len(self.range_data)} range measurements")
        
    def setup_visualization(self):
        """Setup the matplotlib figure and axes"""
        self.fig, (self.ax_main, self.ax_sensor) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Main trajectory plot
        self.ax_main.set_aspect('equal')
        self.ax_main.set_title('Robot Animation - Live Trajectory', fontsize=14, fontweight='bold')
        self.ax_main.set_xlabel('X Position (m)')
        self.ax_main.set_ylabel('Y Position (m)')
        self.ax_main.grid(True, alpha=0.3)
        
        # Sensor data plot
        self.ax_sensor.set_title('LiDAR Range Measurements', fontsize=14, fontweight='bold')
        self.ax_sensor.set_xlabel('Measurement Angle (degrees)')
        self.ax_sensor.set_ylabel('Range (m)')
        self.ax_sensor.grid(True, alpha=0.3)
        
        # Set axis limits
        x_margin = (self.odom_data['x'].max() - self.odom_data['x'].min()) * 0.1
        y_margin = (self.odom_data['y'].max() - self.odom_data['y'].min()) * 0.1
        
        self.ax_main.set_xlim(self.odom_data['x'].min() - x_margin, 
                             self.odom_data['x'].max() + x_margin)
        self.ax_main.set_ylim(self.odom_data['y'].min() - y_margin, 
                             self.odom_data['y'].max() + y_margin)
        
        # Initialize plot elements
        self.trajectory_line, = self.ax_main.plot([], [], 'b-', linewidth=2, alpha=0.7, label='Trajectory')
        self.robot_body = Circle((0, 0), 0.1, color='red', alpha=0.8)
        self.robot_direction = self.ax_main.arrow(0, 0, 0, 0, head_width=0.05, head_length=0.05, 
                                                 fc='darkred', ec='darkred')
        self.ax_main.add_patch(self.robot_body)
        
        # Sensor range visualization
        self.sensor_lines = []
        self.sensor_points = []
        
        # Range data plot
        self.range_plot, = self.ax_sensor.plot([], [], 'g-', linewidth=2, label='Current Scan')
        self.ax_sensor.set_xlim(0, 360)
        self.ax_sensor.set_ylim(0, 5)  # Assuming max range of 5m
        
        # Add legends
        self.ax_main.legend(loc='upper right')
        self.ax_sensor.legend(loc='upper right')
        
        # Add info text
        self.info_text = self.fig.text(0.02, 0.95, '', fontsize=10, verticalalignment='top',
                                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
    def create_robot_shape(self, x, y, theta):
        """Create robot body shape (rectangular with direction indicator)"""
        # Robot dimensions
        length = 0.15
        width = 0.1
        
        # Robot corners in local frame
        corners = np.array([
            [-length/2, -width/2],
            [length/2, -width/2], 
            [length/2, width/2],
            [-length/2, width/2]
        ])
        
        # Rotation matrix
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        rotation_matrix = np.array([[cos_theta, -sin_theta],
                                   [sin_theta, cos_theta]])
        
        # Transform corners to world frame
        rotated_corners = corners @ rotation_matrix.T
        world_corners = rotated_corners + np.array([x, y])
        
        return world_corners
        
    def get_sensor_endpoints(self, x, y, theta, ranges):
        """Calculate sensor ray endpoints"""
        if len(ranges) == 0:
            return [], []
        
        # Assume 360-degree LiDAR with equal angular spacing
        num_rays = len(ranges)
        angles = np.linspace(0, 2*np.pi, num_rays, endpoint=False)
        
        # Transform angles to world frame
        world_angles = angles + theta
        
        # Calculate endpoints
        end_x = x + np.array(ranges) * np.cos(world_angles)
        end_y = y + np.array(ranges) * np.sin(world_angles)
        
        return end_x, end_y
        
    def animate_frame(self, frame):
        """Animation function called for each frame"""
        if frame >= len(self.odom_data):
            return []
        
        # Get current robot state
        current_pose = self.odom_data.iloc[frame]
        x, y, theta = current_pose['x'], current_pose['y'], current_pose['theta']
        t = current_pose['t']
        
        # Update trajectory
        traj_x = self.odom_data['x'][:frame+1]
        traj_y = self.odom_data['y'][:frame+1]
        self.trajectory_line.set_data(traj_x, traj_y)
        
        # Update robot position
        self.robot_body.center = (x, y)
        
        # Remove old arrow and create new one
        if hasattr(self, 'robot_arrow'):
            self.robot_arrow.remove()
        
        # Direction arrow
        arrow_length = 0.15
        dx = arrow_length * np.cos(theta)
        dy = arrow_length * np.sin(theta)
        self.robot_arrow = self.ax_main.arrow(x, y, dx, dy, 
                                            head_width=0.03, head_length=0.03,
                                            fc='darkred', ec='darkred', alpha=0.9)
        
        # Clear previous sensor lines
        for line in self.sensor_lines:
            line.remove()
        for point in self.sensor_points:
            point.remove()
        self.sensor_lines.clear()
        self.sensor_points.clear()
        
        # Get current sensor data
        if frame < len(self.range_data):
            current_ranges = self.range_data.iloc[frame]
            # The range data has only 't' and 'range' columns, so we need to simulate multiple rays
            if 'range' in current_ranges and not pd.isna(current_ranges['range']):
                # Simulate 360-degree LiDAR by creating multiple rays around the single range measurement
                base_range = current_ranges['range']
                # Create 36 rays (every 10 degrees) with some variation
                ranges = []
                for i in range(36):
                    # Add some noise to simulate realistic LiDAR
                    noise = np.random.normal(0, 0.05)  # 5cm standard deviation
                    simulated_range = max(0.1, base_range + noise)  # Minimum 10cm
                    ranges.append(simulated_range)
            else:
                ranges = []
            
            if ranges:
                # Limit number of rays for performance (more aggressive for GIF)
                max_rays = 12  # Show every 30 degrees for memory efficiency
                if len(ranges) > max_rays:
                    step = len(ranges) // max_rays
                    ranges = ranges[::step]
                
                # Get sensor endpoints
                end_x, end_y = self.get_sensor_endpoints(x, y, theta, ranges)
                
                # Draw sensor rays
                for i, (ex, ey, r) in enumerate(zip(end_x, end_y, ranges)):
                    # Color based on distance (closer = red, farther = green)
                    color = plt.cm.RdYlGn(min(r / 3.0, 1.0))  # Normalize to 3m max
                    alpha = 0.6 if i % 3 == 0 else 0.3  # Show every 3rd ray prominently
                    
                    line = self.ax_main.plot([x, ex], [y, ey], color=color, alpha=alpha, linewidth=1)[0]
                    self.sensor_lines.append(line)
                    
                    # Add endpoint dots for prominent rays
                    if i % 3 == 0:
                        point = self.ax_main.plot(ex, ey, 'o', color=color, markersize=3, alpha=0.8)[0]
                        self.sensor_points.append(point)
                
                # Update range plot
                angles_deg = np.linspace(0, 360, len(ranges), endpoint=False)
                self.range_plot.set_data(angles_deg, ranges)
                
                # Update sensor plot axis limits dynamically
                if ranges:
                    self.ax_sensor.set_ylim(0, max(max(ranges) * 1.1, 1.0))
        
        # Update info text
        info_str = f"Frame: {frame}/{len(self.odom_data)-1}\n"
        info_str += f"Time: {t:.2f}s\n"
        info_str += f"Position: ({x:.3f}, {y:.3f})m\n"
        info_str += f"Orientation: {np.degrees(theta):.1f}°\n"
        if frame < len(self.range_data) and 'ranges' in locals():
            info_str += f"Sensor rays: {len(ranges)}\n"
            info_str += f"Min range: {min(ranges):.2f}m\n"
            info_str += f"Max range: {max(ranges):.2f}m"
        
        self.info_text.set_text(info_str)
        
        return [self.trajectory_line, self.robot_body, self.range_plot]
    
    def create_animation(self, interval=50, save_gif=False):
        """Create and run the animation"""
        print("Creating robot animation...")
        
        # Create animation
        self.anim = animation.FuncAnimation(
            self.fig, self.animate_frame, frames=len(self.odom_data),
            interval=interval, blit=False, repeat=True
        )
        
        # Add animation controls text
        control_text = "Controls: Space=Pause/Play, Left/Right=Step, R=Restart"
        self.fig.text(0.5, 0.02, control_text, ha='center', fontsize=10,
                     bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        # Add keyboard controls
        def on_key(event):
            if event.key == ' ':  # Space bar
                if self.anim.running:
                    self.anim.pause()
                else:
                    self.anim.resume()
            elif event.key == 'right':
                self.anim.pause()
                self.animate_frame((self.anim.frame_seq._position + 1) % len(self.odom_data))
                self.fig.canvas.draw()
            elif event.key == 'left':
                self.anim.pause()
                self.animate_frame((self.anim.frame_seq._position - 1) % len(self.odom_data))
                self.fig.canvas.draw()
            elif event.key == 'r':
                self.anim.frame_seq = self.anim.new_frame_seq()
                self.anim.resume()
        
        self.fig.canvas.mpl_connect('key_press_event', on_key)
        
        if save_gif:
            print("Saving animation as GIF (this may take a while)...")
            # Use lower quality settings for memory efficiency
            self.anim.save('robot_animation.gif', writer='pillow', fps=10, dpi=80)
            print("Animation saved as robot_animation.gif")
        
        plt.tight_layout()
        plt.show()
        
        return self.anim

def main():
    """Main function to run the robot animation"""
    # Set data path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_path = os.path.join(script_dir, 'data')
    
    if not os.path.exists(data_path):
        print(f"Error: Data directory not found at {data_path}")
        print("Please ensure the 'data' folder is in the same directory as this script")
        return
    
    print("="*60)
    print("Robot Animation Visualizer")
    print("="*60)
    print("Loading data and creating animation...")
    
    try:
        # Create animator
        animator = RobotAnimator(data_path)
        
        # Ask user for options
        print("\nAnimation Options:")
        print("1. Play animation in window")
        print("2. Play animation and save as GIF")
        
        choice = input("Choose option (1 or 2): ").strip()
        save_gif = choice == '2'
        
        if save_gif:
            print("Note: Saving GIF will take several minutes...")
        
        # Create and run animation
        anim = animator.create_animation(interval=100, save_gif=save_gif)
        
        print("\nAnimation Controls:")
        print("- Space bar: Pause/Resume")
        print("- Left/Right arrows: Step frame by frame")
        print("- R: Restart animation")
        print("- Close window to exit")
        
    except Exception as e:
        print(f"Error creating animation: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
