import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import plotly.graph_objects as go
import plotly.offline as pyo
from plotly.subplots import make_subplots
import seaborn as sns

class Enhanced3DVisualizations:
    def __init__(self, sensor_model=None, motion_model=None, slam=None):
        self.sensor_model = sensor_model
        self.motion_model = motion_model
        self.slam = slam
        
    def create_3d_sensor_analysis(self):
        """Create 3D sensor model visualization"""
        if not self.sensor_model:
            return
            
        # Prepare 3D data for all measurements
        true_distances = []
        measured_distances = []
        measurement_indices = []
        
        for true_dist in sorted(self.sensor_model.calibration_data.keys()):
            measurements = self.sensor_model.calibration_data[true_dist]
            for i, measurement in enumerate(measurements):
                true_distances.append(true_dist)
                measured_distances.append(measurement)
                measurement_indices.append(i)
        
        # Create 3D scatter plot
        fig = go.Figure()
        
        # Add measurement points
        fig.add_trace(go.Scatter3d(
            x=true_distances,
            y=measured_distances,
            z=measurement_indices,
            mode='markers',
            marker=dict(
                size=4,
                color=np.array(measured_distances) - np.array(true_distances),  # Color by error
                colorscale='RdYlBu',
                colorbar=dict(title="Error (m)"),
                opacity=0.8
            ),
            name='Measurements',
            hovertemplate='True: %{x:.3f}m<br>Measured: %{y:.3f}m<br>Sample: %{z}<br>Error: %{marker.color:.4f}m<extra></extra>'
        ))
        
        # Add perfect measurement plane
        true_range = np.linspace(min(true_distances), max(true_distances), 20)
        z_range = np.linspace(0, max(measurement_indices), 20)
        true_mesh, z_mesh = np.meshgrid(true_range, z_range)
        
        fig.add_trace(go.Surface(
            x=true_mesh,
            y=true_mesh,  # Perfect measurements (y=x)
            z=z_mesh,
            opacity=0.3,
            colorscale='Greys',
            showscale=False,
            name='Perfect Measurements'
        ))
        
        fig.update_layout(
            title="3D LiDAR Sensor Calibration Analysis",
            scene=dict(
                xaxis_title="True Distance (m)",
                yaxis_title="Measured Distance (m)",
                zaxis_title="Sample Index",
                camera=dict(eye=dict(x=1.5, y=1.5, z=1.5))
            ),
            height=700
        )
        
        pyo.plot(fig, filename='/Users/ali/Documents/roboex/Sensor Model/sensor_3d_analysis.html', auto_open=False)
        
    def create_3d_motion_trajectory(self):
        """Create enhanced 3D motion trajectory visualization"""
        if not self.motion_model:
            return
            
        data = self.motion_model.motion_data.copy()
        
        # Calculate velocity and acceleration
        data['vx'] = data['x'].diff() / data['t'].diff()
        data['vy'] = data['y'].diff() / data['t'].diff()
        data['speed'] = np.sqrt(data['vx']**2 + data['vy']**2)
        data['acceleration'] = data['speed'].diff() / data['t'].diff()
        data = data.dropna()
        
        fig = go.Figure()
        
        # Main trajectory with time as Z-axis
        fig.add_trace(go.Scatter3d(
            x=data['x'],
            y=data['y'],
            z=data['t'],
            mode='lines+markers',
            line=dict(
                color=data['speed'],
                colorscale='Viridis',
                width=6,
                colorbar=dict(title="Speed (m/s)", x=1.1)
            ),
            marker=dict(
                size=4,
                color=data['acceleration'],
                colorscale='RdBu',
                colorbar=dict(title="Acceleration (m/s²)", x=1.2)
            ),
            name='Motion Trajectory',
            hovertemplate='X: %{x:.3f}m<br>Y: %{y:.3f}m<br>Time: %{z:.2f}s<br>Speed: %{marker.color:.4f}m/s<extra></extra>'
        ))
        
        # Add orientation vectors at key points
        sample_indices = np.linspace(0, len(data)-1, min(20, len(data)), dtype=int)
        for i in sample_indices:
            row = data.iloc[i]
            # Create orientation arrow
            arrow_length = 0.5
            end_x = row['x'] + arrow_length * np.cos(row['theta'])
            end_y = row['y'] + arrow_length * np.sin(row['theta'])
            
            fig.add_trace(go.Scatter3d(
                x=[row['x'], end_x],
                y=[row['y'], end_y],
                z=[row['t'], row['t']],
                mode='lines',
                line=dict(color='red', width=3),
                showlegend=False,
                hoverinfo='skip'
            ))
        
        # Add start and end markers
        fig.add_trace(go.Scatter3d(
            x=[data['x'].iloc[0]], y=[data['y'].iloc[0]], z=[data['t'].iloc[0]],
            mode='markers',
            marker=dict(size=15, color='green', symbol='diamond'),
            name='Start',
            hovertemplate='START<br>X: %{x:.3f}m<br>Y: %{y:.3f}m<br>Time: %{z:.2f}s<extra></extra>'
        ))
        
        fig.add_trace(go.Scatter3d(
            x=[data['x'].iloc[-1]], y=[data['y'].iloc[-1]], z=[data['t'].iloc[-1]],
            mode='markers',
            marker=dict(size=15, color='red', symbol='diamond'),
            name='End',
            hovertemplate='END<br>X: %{x:.3f}m<br>Y: %{y:.3f}m<br>Time: %{z:.2f}s<extra></extra>'
        ))
        
        fig.update_layout(
            title="3D Motion Trajectory with Speed and Acceleration",
            scene=dict(
                xaxis_title="X Position (m)",
                yaxis_title="Y Position (m)",
                zaxis_title="Time (s)",
                camera=dict(eye=dict(x=1.5, y=1.5, z=1.5))
            ),
            height=700
        )
        
        pyo.plot(fig, filename='/Users/ali/Documents/roboex/Motion Model/motion_3d_enhanced.html', auto_open=False)
        
    def create_3d_slam_analysis(self):
        """Create 3D SLAM trajectory visualization with uncertainty"""
        if not self.slam:
            return
            
        poses = self.slam.poses
        n_poses = len(poses)
        
        # Create time-based trajectory
        fig = go.Figure()
        
        # Calculate trajectory metrics for coloring
        velocities = np.zeros(n_poses)
        uncertainties = np.zeros(n_poses)
        
        for i in range(1, n_poses):
            # Velocity approximation
            dx = poses[i, 0] - poses[i-1, 0]
            dy = poses[i, 1] - poses[i-1, 1]
            velocities[i] = np.sqrt(dx**2 + dy**2)
            
            # Uncertainty approximation (orientation change rate)
            dtheta = abs(poses[i, 2] - poses[i-1, 2])
            if dtheta > np.pi:
                dtheta = 2*np.pi - dtheta
            uncertainties[i] = dtheta
        
        # Main trajectory
        fig.add_trace(go.Scatter3d(
            x=poses[:, 0],
            y=poses[:, 1], 
            z=np.arange(n_poses),  # Time as Z-axis
            mode='lines+markers',
            line=dict(
                color=velocities,
                colorscale='Viridis',
                width=4,
                colorbar=dict(title="Velocity (m/step)", x=1.1)
            ),
            marker=dict(
                size=3,
                color=uncertainties,
                colorscale='Reds',
                colorbar=dict(title="Uncertainty (rad)", x=1.2)
            ),
            name='SLAM Trajectory',
            hovertemplate='X: %{x:.3f}m<br>Y: %{y:.3f}m<br>Step: %{z}<br>Velocity: %{marker.color:.4f}<extra></extra>'
        ))
        
        # Add start and end markers
        fig.add_trace(go.Scatter3d(
            x=[poses[0, 0]], y=[poses[0, 1]], z=[0],
            mode='markers',
            marker=dict(size=15, color='green', symbol='diamond'),
            name='Start',
            hovertemplate='START<br>X: %{x:.3f}m<br>Y: %{y:.3f}m<extra></extra>'
        ))
        
        fig.add_trace(go.Scatter3d(
            x=[poses[-1, 0]], y=[poses[-1, 1]], z=[n_poses-1],
            mode='markers',
            marker=dict(size=15, color='red', symbol='diamond'),
            name='End',
            hovertemplate='END<br>X: %{x:.3f}m<br>Y: %{y:.3f}m<extra></extra>'
        ))
        
        # Add loop closure connections in 3D
        if hasattr(self.slam, 'loop_closure_edges') and len(self.slam.loop_closure_edges) > 0:
            for i, edge in enumerate(self.slam.loop_closure_edges[:10]):  # Show first 10
                from_idx, to_idx = edge['from'], edge['to']
                if from_idx < n_poses and to_idx < n_poses:
                    fig.add_trace(go.Scatter3d(
                        x=[poses[from_idx, 0], poses[to_idx, 0]],
                        y=[poses[from_idx, 1], poses[to_idx, 1]],
                        z=[from_idx, to_idx],
                        mode='lines',
                        line=dict(color='red', width=2, dash='dash'),
                        name=f'Loop Closure {i+1}' if i < 3 else None,
                        showlegend=i < 3,
                        hovertemplate='Loop Closure<br>From: %{z[0]}<br>To: %{z[1]}<extra></extra>'
                    ))
        
        fig.update_layout(
            title="3D SLAM Trajectory Analysis",
            scene=dict(
                xaxis_title="X Position (m)",
                yaxis_title="Y Position (m)", 
                zaxis_title="Time Step",
                camera=dict(eye=dict(x=1.5, y=1.5, z=1.5))
            ),
            height=700
        )
        
        pyo.plot(fig, filename='/Users/ali/Documents/roboex/SLAM/slam_3d_trajectory.html', auto_open=False)
        
    def create_3d_occupancy_grid(self):
        """Create 3D occupancy grid visualization"""
        if not self.slam or not hasattr(self.slam, 'occupancy_grid') or self.slam.occupancy_grid is None:
            return
            
        grid = self.slam.occupancy_grid
        height, width = grid.shape
        
        # Create coordinate meshes
        x = np.arange(width)
        y = np.arange(height)
        X, Y = np.meshgrid(x, y)
        
        # Create 3D surface plot
        fig = go.Figure()
        
        # Add occupancy surface
        fig.add_trace(go.Surface(
            x=X, y=Y, z=grid,
            colorscale='Viridis',
            colorbar=dict(title="Occupancy Probability"),
            hovertemplate='Grid X: %{x}<br>Grid Y: %{y}<br>Occupancy: %{z:.3f}<extra></extra>'
        ))
        
        # Add trajectory projection on the grid
        if hasattr(self.slam, 'poses'):
            poses = self.slam.poses
            # Convert world coordinates to grid coordinates (simplified)
            if poses[:, 0].max() != poses[:, 0].min() and poses[:, 1].max() != poses[:, 1].min():
                grid_x = (poses[:, 0] - poses[:, 0].min()) / (poses[:, 0].max() - poses[:, 0].min()) * (width - 1)
                grid_y = (poses[:, 1] - poses[:, 1].min()) / (poses[:, 1].max() - poses[:, 1].min()) * (height - 1)
                
                fig.add_trace(go.Scatter3d(
                    x=grid_x, y=grid_y, z=np.ones(len(poses)) * grid.max() * 1.1,
                    mode='lines+markers',
                    line=dict(color='red', width=4),
                    marker=dict(size=3, color='red'),
                    name='Robot Trajectory',
                    hovertemplate='Trajectory Point<br>Grid X: %{x:.1f}<br>Grid Y: %{y:.1f}<extra></extra>'
                ))
        
        fig.update_layout(
            title="3D Occupancy Grid with Robot Trajectory",
            scene=dict(
                xaxis_title="Grid X",
                yaxis_title="Grid Y",
                zaxis_title="Occupancy Probability",
                camera=dict(eye=dict(x=1.5, y=1.5, z=1.5))
            ),
            height=700
        )
        
        pyo.plot(fig, filename='/Users/ali/Documents/roboex/SLAM/occupancy_grid_3d.html', auto_open=False)
        
    def create_3d_error_analysis(self):
        """Create 3D error analysis visualization"""
        fig = make_subplots(
            rows=2, cols=2,
            specs=[[{"type": "scatter3d"}, {"type": "scatter3d"}],
                   [{"type": "scatter3d"}, {"type": "scatter3d"}]],
            subplot_titles=('Sensor Error 3D', 'Motion Error 3D', 'SLAM Uncertainty 3D', 'Combined Analysis'),
            vertical_spacing=0.1,
            horizontal_spacing=0.1
        )
        
        # Sensor error 3D
        if self.sensor_model:
            true_distances = []
            errors = []
            sample_nums = []
            
            for true_dist in sorted(self.sensor_model.calibration_data.keys()):
                measurements = self.sensor_model.calibration_data[true_dist]
                for i, measurement in enumerate(measurements):
                    true_distances.append(true_dist)
                    errors.append(measurement - true_dist)
                    sample_nums.append(i)
            
            fig.add_trace(go.Scatter3d(
                x=true_distances, y=errors, z=sample_nums,
                mode='markers',
                marker=dict(size=3, color=errors, colorscale='RdBu'),
                name='Sensor Errors',
                hovertemplate='Distance: %{x:.3f}m<br>Error: %{y:.4f}m<br>Sample: %{z}<extra></extra>'
            ), row=1, col=1)
        
        # Motion error 3D
        if self.motion_model:
            data = self.motion_model.motion_data.copy()
            data['dx'] = data['x'].diff()
            data['dy'] = data['y'].diff()
            data = data.dropna()
            
            # Calculate position errors from expected straight line motion
            expected_dx = np.mean(data['dx'])
            expected_dy = np.mean(data['dy'])
            error_x = data['dx'] - expected_dx
            error_y = data['dy'] - expected_dy
            
            fig.add_trace(go.Scatter3d(
                x=error_x, y=error_y, z=data['t'],
                mode='markers',
                marker=dict(size=3, color=data['t'], colorscale='Viridis'),
                name='Motion Errors',
                hovertemplate='X Error: %{x:.4f}m<br>Y Error: %{y:.4f}m<br>Time: %{z:.2f}s<extra></extra>'
            ), row=1, col=2)
        
        # SLAM uncertainty 3D
        if self.slam:
            poses = self.slam.poses
            # Calculate pose-to-pose variations as uncertainty proxy
            pose_variations = np.diff(poses, axis=0)
            uncertainties = np.linalg.norm(pose_variations, axis=1)
            
            fig.add_trace(go.Scatter3d(
                x=poses[1:, 0], y=poses[1:, 1], z=uncertainties,
                mode='markers',
                marker=dict(size=4, color=uncertainties, colorscale='Reds'),
                name='SLAM Uncertainty',
                hovertemplate='X: %{x:.3f}m<br>Y: %{y:.3f}m<br>Uncertainty: %{z:.4f}<extra></extra>'
            ), row=2, col=1)
        
        # Combined analysis
        if self.sensor_model and self.motion_model and self.slam:
            # Create a combined metric visualization
            sensor_error_mean = np.mean([abs(e) for e in errors]) if 'errors' in locals() else 0
            motion_error_mean = np.mean([abs(ex) + abs(ey) for ex, ey in zip(error_x, error_y)]) if 'error_x' in locals() else 0
            slam_uncertainty_mean = np.mean(uncertainties) if 'uncertainties' in locals() else 0
            
            metrics = ['Sensor', 'Motion', 'SLAM']
            values = [sensor_error_mean, motion_error_mean, slam_uncertainty_mean]
            colors = ['red', 'green', 'blue']
            
            fig.add_trace(go.Scatter3d(
                x=[0, 1, 2], y=values, z=[0, 0, 0],
                mode='markers+text',
                marker=dict(size=15, color=colors),
                text=metrics,
                textposition="top center",
                name='Error Comparison',
                hovertemplate='System: %{text}<br>Mean Error: %{y:.4f}<extra></extra>'
            ), row=2, col=2)
        
        fig.update_layout(
            title="3D Error Analysis Dashboard",
            height=800
        )
        
        pyo.plot(fig, filename='/Users/ali/Documents/roboex/error_analysis_3d.html', auto_open=False)
        
    def generate_all_3d_visualizations(self):
        """Generate all 3D visualizations"""
        print("Creating 3D sensor analysis...")
        self.create_3d_sensor_analysis()
        
        print("Creating 3D motion trajectory...")
        self.create_3d_motion_trajectory()
        
        print("Creating 3D SLAM analysis...")
        self.create_3d_slam_analysis()
        
        print("Creating 3D occupancy grid...")
        self.create_3d_occupancy_grid()
        
        print("Creating 3D error analysis...")
        self.create_3d_error_analysis()
        
        print("\n=== 3D Visualizations Generated ===")
        print("Files created:")
        print("- Sensor Model/sensor_3d_analysis.html")
        print("- Motion Model/motion_3d_enhanced.html") 
        print("- SLAM/slam_3d_trajectory.html")
        print("- SLAM/occupancy_grid_3d.html")
        print("- error_analysis_3d.html")
