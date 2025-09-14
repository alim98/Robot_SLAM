import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

class GraphOptimizationSLAM:
    def __init__(self, sensor_model=None, motion_model=None):
        self.poses = []  # Robot poses (x, y, theta)
        self.landmarks = []  # Map landmarks
        self.edges = []  # Graph edges (constraints)
        self.information_matrices = []  # Information matrices for edges
        
        # Sensor and motion models
        self.sensor_model = sensor_model
        self.motion_model = motion_model
        
        # Data
        self.odom_data = None
        self.range_data = None
        self.loop_closures = []
        
        # Map representation
        self.occupancy_grid = None
        self.grid_resolution = 0.05  # 5cm resolution
        self.grid_size = None
        
    def load_maze_data(self, data_path):
        """Load maze navigation data"""
        maze_path = os.path.join(data_path, 'maze')
        
        # Load odometry data
        odom_file = os.path.join(maze_path, 'odom_data.csv')
        self.odom_data = pd.read_csv(odom_file)
        
        # Load range data
        range_file = os.path.join(maze_path, 'range_data.csv')
        self.range_data = pd.read_csv(range_file)
        
        # Extract loop closures
        self.extract_loop_closures()
        
    def extract_loop_closures(self):
        """Extract loop closure events from odometry data"""
        loop_closure_indices = self.odom_data[self.odom_data['loop_closure'] == 1].index.tolist()
        
        # Group consecutive loop closure indices
        if loop_closure_indices:
            groups = []
            current_group = [loop_closure_indices[0]]
            
            for i in range(1, len(loop_closure_indices)):
                if loop_closure_indices[i] - loop_closure_indices[i-1] <= 10:  # Within 10 samples
                    current_group.append(loop_closure_indices[i])
                else:
                    groups.append(current_group)
                    current_group = [loop_closure_indices[i]]
            groups.append(current_group)
            
            # Store loop closure events
            for group in groups:
                start_idx = group[0]
                end_idx = group[-1]
                self.loop_closures.append({
                    'start_idx': start_idx,
                    'end_idx': end_idx,
                    'start_pose': (self.odom_data.iloc[start_idx]['x'], 
                                 self.odom_data.iloc[start_idx]['y'], 
                                 self.odom_data.iloc[start_idx]['theta']),
                    'end_pose': (self.odom_data.iloc[end_idx]['x'], 
                               self.odom_data.iloc[end_idx]['y'], 
                               self.odom_data.iloc[end_idx]['theta'])
                })
    
    def initialize_poses(self):
        """Initialize robot poses from odometry"""
        self.poses = []
        for _, row in self.odom_data.iterrows():
            self.poses.append([row['x'], row['y'], row['theta']])
        self.poses = np.array(self.poses)
    
    def create_odometry_edges(self):
        """Create edges from odometry constraints"""
        self.edges = []
        self.information_matrices = []
        
        # Motion model covariance
        if self.motion_model:
            motion_cov = np.diag([
                self.motion_model.variance['x'],
                self.motion_model.variance['y'], 
                self.motion_model.variance['theta']
            ])
        else:
            # Default covariance
            motion_cov = np.diag([1e-6, 1e-6, 1e-8])
        
        motion_info = np.linalg.inv(motion_cov)
        
        # Add odometry edges between consecutive poses
        for i in range(len(self.poses) - 1):
            # Calculate relative transformation
            dx = self.poses[i+1, 0] - self.poses[i, 0]
            dy = self.poses[i+1, 1] - self.poses[i, 1]
            dtheta = self.poses[i+1, 2] - self.poses[i, 2]
            
            # Normalize angle
            dtheta = self.normalize_angle(dtheta)
            
            edge = {
                'type': 'odometry',
                'from': i,
                'to': i + 1,
                'measurement': np.array([dx, dy, dtheta]),
                'information': motion_info.copy()
            }
            
            self.edges.append(edge)
    
    def create_loop_closure_edges(self):
        """Create edges from loop closure constraints"""
        # Loop closure covariance (more certain than odometry)
        loop_cov = np.diag([1e-4, 1e-4, 1e-6])
        loop_info = np.linalg.inv(loop_cov)
        
        for lc in self.loop_closures:
            start_idx = lc['start_idx']
            end_idx = lc['end_idx']
            
            if abs(end_idx - start_idx) > 100:  # Significant loop closure
                # Calculate expected relative transformation (should be small)
                dx = self.poses[end_idx, 0] - self.poses[start_idx, 0]
                dy = self.poses[end_idx, 1] - self.poses[start_idx, 1]
                dtheta = self.poses[end_idx, 2] - self.poses[start_idx, 2]
                dtheta = self.normalize_angle(dtheta)
                
                edge = {
                    'type': 'loop_closure',
                    'from': start_idx,
                    'to': end_idx,
                    'measurement': np.array([dx, dy, dtheta]),
                    'information': loop_info.copy()
                }
                
                self.edges.append(edge)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def compute_error(self, edge):
        """Compute error for an edge"""
        i, j = edge['from'], edge['to']
        
        # Current poses
        xi, yi, thi = self.poses[i]
        xj, yj, thj = self.poses[j]
        
        # Expected measurement from current poses
        dx_pred = xj - xi
        dy_pred = yj - yi
        dth_pred = self.normalize_angle(thj - thi)
        
        predicted = np.array([dx_pred, dy_pred, dth_pred])
        
        # Error
        error = edge['measurement'] - predicted
        error[2] = self.normalize_angle(error[2])  # Normalize angle error
        
        return error
    
    def compute_jacobian(self, edge):
        """Compute Jacobian for an edge"""
        i, j = edge['from'], edge['to']
        
        # Jacobian is simple for pose-to-pose constraints
        # Error = measurement - (pose_j - pose_i)
        # d(error)/d(pose_i) = [1, 1, 1]
        # d(error)/d(pose_j) = [-1, -1, -1]
        
        Ji = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])
        
        Jj = np.array([
            [-1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        
        return Ji, Jj
    
    def optimize_graph(self, max_iterations=10, convergence_threshold=1e-4):
        """Perform graph optimization using Gauss-Newton method - implemented from scratch"""
        
        n_poses = len(self.poses)
        n_vars = 3 * n_poses
        
        for iteration in range(max_iterations):
            # Build linearized system Hx = b using dense matrices
            H = np.zeros((n_vars, n_vars))
            b = np.zeros(n_vars)
            
            total_error = 0
            
            for edge in self.edges:
                i, j = edge['from'], edge['to']
                
                # Compute error and Jacobians
                error = self.compute_error(edge)
                Ji, Jj = self.compute_jacobian(edge)
                
                # Information matrix
                info = edge['information']
                
                # Accumulate into H and b
                # H += J^T * Info * J
                # b += J^T * Info * error
                
                # For pose i
                H[3*i:3*i+3, 3*i:3*i+3] += Ji.T @ info @ Ji
                b[3*i:3*i+3] += Ji.T @ info @ error
                
                # For pose j
                H[3*j:3*j+3, 3*j:3*j+3] += Jj.T @ info @ Jj
                b[3*j:3*j+3] += Jj.T @ info @ error
                
                # Cross terms
                H[3*i:3*i+3, 3*j:3*j+3] += Ji.T @ info @ Jj
                H[3*j:3*j+3, 3*i:3*i+3] += Jj.T @ info @ Ji
                
                # Accumulate total error
                total_error += error.T @ info @ error
            
            # Add regularization to prevent singularity
            regularization = 1e-6 * np.eye(n_vars)
            H += regularization
            
            # Fix first pose (gauge constraint)
            H[0:3, :] = 0
            H[0:3, 0:3] = np.eye(3)
            b[0:3] = 0
            
            # Solve linear system using numpy only
            try:
                dx = np.linalg.solve(H, b)
            except np.linalg.LinAlgError:
                # Use pseudo-inverse if singular
                dx = np.linalg.pinv(H) @ b
            
            # Limit update step size to prevent divergence
            max_step = 0.05
            dx_norm = np.linalg.norm(dx)
            if dx_norm > max_step:
                dx = dx * (max_step / dx_norm)
            
            # Update poses
            for i in range(n_poses):
                self.poses[i, 0] += dx[3*i]
                self.poses[i, 1] += dx[3*i+1]
                self.poses[i, 2] += dx[3*i+2]
                self.poses[i, 2] = self.normalize_angle(self.poses[i, 2])
            
            print(f"Iteration {iteration}: error = {total_error:.6f}, dx_norm = {dx_norm:.8f}")
            
            if dx_norm < convergence_threshold:
                print(f"Converged after {iteration + 1} iterations")
                break
    
    def build_occupancy_grid(self):
        """Build occupancy grid map from range data and optimized poses"""
        # Synchronize range data with poses
        range_times = self.range_data['t'].values
        odom_times = self.odom_data['t'].values
        
        # Find corresponding poses for each range measurement
        synchronized_data = []
        
        for i, range_time in enumerate(range_times):
            # Find closest odometry measurement
            closest_idx = np.argmin(np.abs(odom_times - range_time))
            
            if abs(odom_times[closest_idx] - range_time) < 0.1:  # Within 100ms
                synchronized_data.append({
                    'pose_idx': closest_idx,
                    'range': self.range_data.iloc[i]['range'],
                    'x': self.poses[closest_idx, 0],
                    'y': self.poses[closest_idx, 1],
                    'theta': self.poses[closest_idx, 2]
                })
        
        # Determine grid bounds
        all_x = [d['x'] for d in synchronized_data]
        all_y = [d['y'] for d in synchronized_data]
        
        # Add range measurements to bounds
        for d in synchronized_data:
            # Obstacle position
            obs_x = d['x'] + d['range'] * np.cos(d['theta'])
            obs_y = d['y'] + d['range'] * np.sin(d['theta'])
            all_x.append(obs_x)
            all_y.append(obs_y)
        
        min_x, max_x = min(all_x) - 0.5, max(all_x) + 0.5
        min_y, max_y = min(all_y) - 0.5, max(all_y) + 0.5
        
        # Create grid
        grid_width = int((max_x - min_x) / self.grid_resolution)
        grid_height = int((max_y - min_y) / self.grid_resolution)
        
        self.occupancy_grid = np.ones((grid_height, grid_width)) * 0.5  # Unknown = 0.5
        self.grid_size = (grid_width, grid_height)
        self.grid_origin = (min_x, min_y)
        
        # Fill grid with measurements
        for d in synchronized_data:
            robot_x, robot_y = d['x'], d['y']
            
            # Convert to grid coordinates
            robot_gx = int((robot_x - min_x) / self.grid_resolution)
            robot_gy = int((robot_y - min_y) / self.grid_resolution)
            
            # Obstacle position
            obs_x = robot_x + d['range'] * np.cos(d['theta'])
            obs_y = robot_y + d['range'] * np.sin(d['theta'])
            
            obs_gx = int((obs_x - min_x) / self.grid_resolution)
            obs_gy = int((obs_y - min_y) / self.grid_resolution)
            
            # Mark free space along ray
            self.bresenham_line(robot_gx, robot_gy, obs_gx, obs_gy, free_value=0.2)
            
            # Mark obstacle
            if 0 <= obs_gx < grid_width and 0 <= obs_gy < grid_height:
                self.occupancy_grid[obs_gy, obs_gx] = 0.8  # Occupied
    
    def bresenham_line(self, x0, y0, x1, y1, free_value=0.2):
        """Bresenham's line algorithm to mark free space"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            if 0 <= x < self.grid_size[0] and 0 <= y < self.grid_size[1]:
                if self.occupancy_grid[y, x] == 0.5:  # Only update unknown cells
                    self.occupancy_grid[y, x] = free_value
            
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def subsample_data(self, subsample_factor=10):
        """Subsample data to reduce computational load"""
        print(f"Subsampling data by factor of {subsample_factor}...")
        
        # Subsample odometry data
        self.odom_data = self.odom_data.iloc[::subsample_factor].reset_index(drop=True)
        
        # Update loop closure indices to match subsampled data
        new_loop_closures = []
        
        for lc in self.loop_closures:
            # Map original indices to subsampled indices
            start_new = lc['start_idx'] // subsample_factor
            end_new = lc['end_idx'] // subsample_factor
            
            # Ensure indices are within bounds
            start_new = min(start_new, len(self.odom_data) - 1)
            end_new = min(end_new, len(self.odom_data) - 1)
            
            if abs(end_new - start_new) > 5:  # Keep significant loop closures
                new_loop_closures.append({
                    'start_idx': start_new,
                    'end_idx': end_new,
                    'start_pose': (self.odom_data.iloc[start_new]['x'], 
                                 self.odom_data.iloc[start_new]['y'], 
                                 self.odom_data.iloc[start_new]['theta']),
                    'end_pose': (self.odom_data.iloc[end_new]['x'], 
                               self.odom_data.iloc[end_new]['y'], 
                               self.odom_data.iloc[end_new]['theta'])
                })
        
        self.loop_closures = new_loop_closures
        print(f"Reduced to {len(self.odom_data)} poses, {len(self.loop_closures)} loop closures")

    def run_slam(self, data_path):
        """Run complete SLAM pipeline"""
        print("Loading maze data...")
        self.load_maze_data(data_path)
        
        # Subsample for computational efficiency
        self.subsample_data(subsample_factor=20)  # Use every 20th pose
        
        print("Initializing poses...")
        self.initialize_poses()
        
        print("Creating odometry edges...")
        self.create_odometry_edges()
        
        print("Creating loop closure edges...")
        self.create_loop_closure_edges()
        
        print(f"Graph: {len(self.poses)} poses, {len(self.edges)} edges")
        print(f"Loop closures: {len(self.loop_closures)}")
        
        print("Optimizing graph...")
        self.optimize_graph(max_iterations=10)  # Use custom optimization
        
        print("Building occupancy grid...")
        self.build_occupancy_grid()
        
        return self.poses, self.occupancy_grid
    
    def plot_slam_results(self, show_stages=True):
        """Plot SLAM results with map evolution"""
        if show_stages:
            self.plot_map_evolution()
        
        self.plot_final_results()
    
    def plot_map_evolution(self):
        """Plot map evolution in 5 stages"""
        n_poses = len(self.poses)
        stages = [int(n_poses * i / 5) for i in range(1, 6)]
        
        fig, axes = plt.subplots(1, 5, figsize=(20, 4))
        
        for i, stage_idx in enumerate(stages):
            ax = axes[i]
            
            # Plot trajectory up to this stage
            ax.plot(self.poses[:stage_idx, 0], self.poses[:stage_idx, 1], 'b-', alpha=0.7, linewidth=1)
            ax.plot(self.poses[0, 0], self.poses[0, 1], 'go', markersize=8, label='Start')
            ax.plot(self.poses[stage_idx-1, 0], self.poses[stage_idx-1, 1], 'ro', markersize=8, label='Current')
            
            # Mark loop closures up to this stage
            for lc in self.loop_closures:
                if lc['end_idx'] <= stage_idx:
                    start_pose = self.poses[lc['start_idx']]
                    end_pose = self.poses[lc['end_idx']]
                    ax.plot([start_pose[0], end_pose[0]], [start_pose[1], end_pose[1]], 'r--', alpha=0.5)
            
            ax.set_title(f'Stage {i+1}: {stage_idx} poses')
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.grid(True)
            ax.axis('equal')
            if i == 0:
                ax.legend()
        
        plt.tight_layout()
        plt.savefig('/Users/ali/Documents/roboex/SLAM/map_evolution.png', dpi=300, bbox_inches='tight')
        # plt.show()
    
    def plot_final_results(self):
        """Plot final SLAM results"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Plot 1: Optimized trajectory with loop closures
        ax1.plot(self.poses[:, 0], self.poses[:, 1], 'b-', alpha=0.7, linewidth=2, label='Optimized trajectory')
        ax1.plot(self.poses[0, 0], self.poses[0, 1], 'go', markersize=10, label='Start position')
        ax1.plot(self.poses[-1, 0], self.poses[-1, 1], 'ro', markersize=10, label='End position')
        
        # Mark loop closures
        for lc in self.loop_closures:
            start_pose = self.poses[lc['start_idx']]
            end_pose = self.poses[lc['end_idx']]
            ax1.plot([start_pose[0], end_pose[0]], [start_pose[1], end_pose[1]], 'r--', alpha=0.7, linewidth=1)
        
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.set_title('Optimized Robot Trajectory')
        ax1.legend()
        ax1.grid(True)
        ax1.axis('equal')
        
        # Plot 2: Occupancy grid map
        if self.occupancy_grid is not None:
            extent = [self.grid_origin[0], self.grid_origin[0] + self.grid_size[0] * self.grid_resolution,
                     self.grid_origin[1], self.grid_origin[1] + self.grid_size[1] * self.grid_resolution]
            
            ax2.imshow(self.occupancy_grid, cmap='gray_r', origin='lower', extent=extent)
            ax2.plot(self.poses[:, 0], self.poses[:, 1], 'b-', alpha=0.8, linewidth=1)
            ax2.plot(self.poses[0, 0], self.poses[0, 1], 'go', markersize=8, label='Start')
            ax2.plot(self.poses[-1, 0], self.poses[-1, 1], 'ro', markersize=8, label='End')
            
            ax2.set_xlabel('X Position (m)')
            ax2.set_ylabel('Y Position (m)')
            ax2.set_title('Final Occupancy Grid Map')
            ax2.legend()
        
        plt.tight_layout()
        plt.savefig('/Users/ali/Documents/roboex/SLAM/final_results.png', dpi=300, bbox_inches='tight')
        # plt.show()

def run_graph_optimization_slam(data_path, sensor_model=None, motion_model=None):
    """Main function to run Graph Optimization SLAM"""
    slam = GraphOptimizationSLAM(sensor_model, motion_model)
    
    # Run SLAM
    poses, occupancy_grid = slam.run_slam(data_path)
    
    # Plot results
    slam.plot_slam_results()
    
    return slam

if __name__ == "__main__":
    data_path = "/Users/ali/Documents/roboex/data"
    slam = run_graph_optimization_slam(data_path)
