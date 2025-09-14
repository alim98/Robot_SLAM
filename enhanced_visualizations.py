#!/usr/bin/env python3
"""
Enhanced Visualizations for RoboEx SLAM Project
Creates additional interactive and detailed plots using plotly and seaborn
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
import plotly.offline as pyo
import os

class EnhancedVisualizations:
    def __init__(self, sensor_model=None, motion_model=None, slam=None):
        self.sensor_model = sensor_model
        self.motion_model = motion_model
        self.slam = slam
        
    def create_interactive_sensor_analysis(self):
        """Create interactive sensor model analysis with plotly"""
        if not self.sensor_model:
            return
            
        # Get sensor statistics
        stats = self.sensor_model.get_overall_statistics()
        
        # Create subplots
        fig = make_subplots(
            rows=2, cols=2,
            subplot_titles=('Measured vs True Distance', 'Bias Analysis', 
                          'Variance Analysis', 'Error Distribution'),
            specs=[[{"secondary_y": False}, {"secondary_y": False}],
                   [{"secondary_y": False}, {"secondary_y": False}]]
        )
        
        # Data preparation
        true_dists = list(self.sensor_model.calibration_data.keys())
        mean_measurements = [np.mean(self.sensor_model.calibration_data[d]) for d in true_dists]
        std_measurements = [np.std(self.sensor_model.calibration_data[d]) for d in true_dists]
        biases = [stats['distance_specific_bias'][d] for d in true_dists]
        variances = [stats['distance_specific_variance'][d] for d in true_dists]
        
        # Plot 1: Measured vs True with error bars
        fig.add_trace(
            go.Scatter(x=true_dists, y=mean_measurements, 
                      error_y=dict(type='data', array=std_measurements),
                      mode='markers+lines', name='Measured',
                      hovertemplate='True: %{x:.3f}m<br>Measured: %{y:.3f}m<extra></extra>'),
            row=1, col=1
        )
        fig.add_trace(
            go.Scatter(x=true_dists, y=true_dists, 
                      mode='lines', name='Perfect (y=x)', line=dict(dash='dash')),
            row=1, col=1
        )
        
        # Plot 2: Bias analysis
        fig.add_trace(
            go.Scatter(x=true_dists, y=biases, mode='markers+lines', 
                      name='Bias', marker=dict(color='red'),
                      hovertemplate='Distance: %{x:.3f}m<br>Bias: %{y:.6f}m<extra></extra>'),
            row=1, col=2
        )
        fig.add_hline(y=0, line_dash="dash", line_color="gray", row=1, col=2)
        
        # Plot 3: Variance analysis
        fig.add_trace(
            go.Scatter(x=true_dists, y=variances, mode='markers+lines', 
                      name='Variance', marker=dict(color='green'),
                      hovertemplate='Distance: %{x:.3f}m<br>Variance: %{y:.8f}m²<extra></extra>'),
            row=2, col=1
        )
        
        # Plot 4: Error distribution histogram
        all_errors = []
        for true_dist in true_dists:
            measurements = self.sensor_model.calibration_data[true_dist]
            errors = measurements - true_dist
            all_errors.extend(errors)
        
        fig.add_trace(
            go.Histogram(x=all_errors, nbinsx=30, name='Error Distribution',
                        hovertemplate='Error: %{x:.4f}m<br>Count: %{y}<extra></extra>'),
            row=2, col=2
        )
        
        # Update layout
        fig.update_layout(
            title_text="Interactive LiDAR Sensor Model Analysis",
            showlegend=True,
            height=800
        )
        
        # Save as HTML
        pyo.plot(fig, filename='/Users/ali/Documents/roboex/Sensor Model/interactive_sensor_analysis.html', auto_open=False)
        
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
        
    def create_interactive_motion_analysis(self):
        """Create interactive motion model analysis"""
        if not self.motion_model:
            return
            
        data = self.motion_model.motion_data.copy()
        
        # Create 3D trajectory plot
        fig = go.Figure()
        
        # Add 3D trajectory
        fig.add_trace(go.Scatter3d(
            x=data['x'], y=data['y'], z=data['t'],
            mode='lines',
            line=dict(color=data['theta'], colorscale='Viridis', width=3),
            name='Robot Trajectory',
            hovertemplate='X: %{x:.4f}m<br>Y: %{y:.4f}m<br>Time: %{z:.2f}s<extra></extra>'
        ))
        
        # Add start and end points
        fig.add_trace(go.Scatter3d(
            x=[data['x'].iloc[0]], y=[data['y'].iloc[0]], z=[data['t'].iloc[0]],
            mode='markers', marker=dict(size=10, color='green'),
            name='Start Position'
        ))
        
        fig.add_trace(go.Scatter3d(
            x=[data['x'].iloc[-1]], y=[data['y'].iloc[-1]], z=[data['t'].iloc[-1]],
            mode='markers', marker=dict(size=10, color='red'),
            name='End Position'
        ))
        
        fig.update_layout(
            title="3D Robot Motion Trajectory (Time vs Position)",
            scene=dict(
                xaxis_title="X Position (m)",
                yaxis_title="Y Position (m)",
                zaxis_title="Time (s)"
            ),
            height=700
        )
        
        pyo.plot(fig, filename='/Users/ali/Documents/roboex/Motion Model/interactive_motion_3d.html', auto_open=False)
        
        # Create motion statistics dashboard
        fig2 = make_subplots(
            rows=2, cols=2,
            subplot_titles=('Position Over Time', 'Velocity Analysis', 
                          'Angular Velocity', 'Motion Increments'),
            specs=[[{"secondary_y": True}, {"secondary_y": False}],
                   [{"secondary_y": False}, {"secondary_y": False}]]
        )
        
        # Calculate velocities
        data['vx'] = data['x'].diff() / data['t'].diff()
        data['vy'] = data['y'].diff() / data['t'].diff()
        data['vtheta'] = data['theta'].diff() / data['t'].diff()
        data['speed'] = np.sqrt(data['vx']**2 + data['vy']**2)
        
        # Plot 1: Position over time
        fig2.add_trace(go.Scatter(x=data['t'], y=data['x'], name='X Position', line=dict(color='red')), row=1, col=1)
        fig2.add_trace(go.Scatter(x=data['t'], y=data['y'], name='Y Position', line=dict(color='blue')), row=1, col=1, secondary_y=True)
        
        # Plot 2: Speed analysis
        fig2.add_trace(go.Scatter(x=data['t'], y=data['speed'], name='Speed', line=dict(color='purple')), row=1, col=2)
        
        # Plot 3: Angular velocity
        fig2.add_trace(go.Scatter(x=data['t'], y=data['vtheta'], name='Angular Velocity', line=dict(color='orange')), row=2, col=1)
        
        # Plot 4: Motion increments distribution
        dx = data['x'].diff().dropna()
        dy = data['y'].diff().dropna()
        
        fig2.add_trace(go.Histogram(x=dx, name='ΔX', opacity=0.7, nbinsx=50), row=2, col=2)
        fig2.add_trace(go.Histogram(x=dy, name='ΔY', opacity=0.7, nbinsx=50), row=2, col=2)
        
        poses = self.slam.poses
        # Convert world coordinates to grid coordinates (simplified)
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

def create_comprehensive_error_analysis(self):
    """Create detailed error analysis plots"""
    # Set style for matplotlib plots
    plt.style.use('seaborn-v0_8')
    
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    
    # Sensor Model Error Analysis
    if self.sensor_model:
        stats = self.sensor_model.get_overall_statistics()
        
        # Plot 1: Bias vs Distance with confidence intervals
        true_dists = list(self.sensor_model.calibration_data.keys())
        biases = [stats['distance_specific_bias'][d] for d in true_dists]
        variances = [stats['distance_specific_variance'][d] for d in true_dists]
        std_devs = [np.sqrt(v) for v in variances]
        
        axes[0, 0].errorbar(true_dists, biases, yerr=std_devs, 
                           fmt='o-', capsize=5, capthick=2)
        axes[0, 0].axhline(y=0, color='r', linestyle='--', alpha=0.7)
        axes[0, 0].set_title('LiDAR Bias vs Distance')
        axes[0, 0].set_xlabel('True Distance (m)')
        axes[0, 0].set_ylabel('Bias (m)')
        axes[0, 0].grid(True, alpha=0.3)
        
        # Plot 2: Measurement precision analysis
        all_measurements = []
        all_true_dists = []
        for true_dist in true_dists:
            measurements = self.sensor_model.calibration_data[true_dist]
            all_measurements.extend(measurements)
            all_true_dists.extend([true_dist] * len(measurements))
        
        df_sensor = pd.DataFrame({
            'true_distance': all_true_dists,
            'measured_distance': all_measurements,
            'error': np.array(all_measurements) - np.array(all_true_dists)
        })
        
        sns.boxplot(data=df_sensor, x='true_distance', y='error', ax=axes[0, 1])
        axes[0, 1].set_title('Measurement Error Distribution by Distance')
        axes[0, 1].set_xlabel('True Distance (m)')
        axes[0, 1].set_ylabel('Error (m)')
        axes[0, 1].tick_params(axis='x', rotation=45)
    
    # Motion Model Error Analysis
    if self.motion_model:
        data = self.motion_model.motion_data.copy()
        
        # Calculate incremental errors
        data['dx'] = data['x'].diff()
        data['dy'] = data['y'].diff()
        data['dtheta'] = data['theta'].diff()
        data = data.dropna()
        
        # Plot 3: Motion increment distributions
        axes[0, 2].hist(data['dx'], bins=50, alpha=0.7, label='ΔX', density=True)
        axes[0, 2].hist(data['dy'], bins=50, alpha=0.7, label='ΔY', density=True)
        axes[0, 2].set_title('Motion Increment Distributions')
        axes[0, 2].set_xlabel('Displacement (m)')
        axes[0, 2].set_ylabel('Density')
        axes[0, 2].legend()
        axes[0, 2].grid(True, alpha=0.3)
        
        # Plot 4: Cumulative error over time
        cumulative_error_x = np.cumsum(np.abs(data['dx'] - np.mean(data['dx'])))
        cumulative_error_y = np.cumsum(np.abs(data['dy'] - np.mean(data['dy'])))
        
        axes[1, 0].plot(data['t'], cumulative_error_x, label='Cumulative X Error')
        axes[1, 0].plot(data['t'], cumulative_error_y, label='Cumulative Y Error')
        axes[1, 0].set_title('Cumulative Odometry Error')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Cumulative Error (m)')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
    
    # SLAM Analysis
    if self.slam:
        poses = self.slam.poses
        
        # Plot 5: Trajectory smoothness analysis
        pose_changes = np.diff(poses, axis=0)
        smoothness = np.linalg.norm(pose_changes, axis=1)
        
        # Sensor Model Error Analysis
        if self.sensor_model:
            stats = self.sensor_model.get_overall_statistics()
            
            # Plot 1: Bias vs Distance with confidence intervals
            true_dists = list(self.sensor_model.calibration_data.keys())
            biases = [stats['distance_specific_bias'][d] for d in true_dists]
            variances = [stats['distance_specific_variance'][d] for d in true_dists]
            std_devs = [np.sqrt(v) for v in variances]
            
            axes[0, 0].errorbar(true_dists, biases, yerr=std_devs, 
                               fmt='o-', capsize=5, capthick=2)
            axes[0, 0].axhline(y=0, color='r', linestyle='--', alpha=0.7)
            axes[0, 0].set_title('LiDAR Bias vs Distance')
            axes[0, 0].set_xlabel('True Distance (m)')
            axes[0, 0].set_ylabel('Bias (m)')
            axes[0, 0].grid(True, alpha=0.3)
            
            # Plot 2: Measurement precision analysis
            all_measurements = []
            all_true_dists = []
            for true_dist in true_dists:
                measurements = self.sensor_model.calibration_data[true_dist]
                all_measurements.extend(measurements)
                all_true_dists.extend([true_dist] * len(measurements))
            
            df_sensor = pd.DataFrame({
                'true_distance': all_true_dists,
                'measured_distance': all_measurements,
                'error': np.array(all_measurements) - np.array(all_true_dists)
            })
            
            sns.boxplot(data=df_sensor, x='true_distance', y='error', ax=axes[0, 1])
            axes[0, 1].set_title('Measurement Error Distribution by Distance')
            axes[0, 1].set_xlabel('True Distance (m)')
            axes[0, 1].set_ylabel('Error (m)')
            axes[0, 1].tick_params(axis='x', rotation=45)
        
        # Motion Model Error Analysis
        if self.motion_model:
            data = self.motion_model.motion_data.copy()
            
            # Calculate incremental errors
            data['dx'] = data['x'].diff()
            data['dy'] = data['y'].diff()
            data['dtheta'] = data['theta'].diff()
            data = data.dropna()
            
            # Plot 3: Motion increment distributions
            axes[0, 2].hist(data['dx'], bins=50, alpha=0.7, label='ΔX', density=True)
            axes[0, 2].hist(data['dy'], bins=50, alpha=0.7, label='ΔY', density=True)
            axes[0, 2].set_title('Motion Increment Distributions')
            axes[0, 2].set_xlabel('Displacement (m)')
            axes[0, 2].set_ylabel('Density')
            axes[0, 2].legend()
            axes[0, 2].grid(True, alpha=0.3)
            
            # Plot 4: Cumulative error over time
            cumulative_error_x = np.cumsum(np.abs(data['dx'] - np.mean(data['dx'])))
            cumulative_error_y = np.cumsum(np.abs(data['dy'] - np.mean(data['dy'])))
            
            axes[1, 0].plot(data['t'], cumulative_error_x, label='Cumulative X Error')
            axes[1, 0].plot(data['t'], cumulative_error_y, label='Cumulative Y Error')
            axes[1, 0].set_title('Cumulative Odometry Error')
            axes[1, 0].set_xlabel('Time (s)')
            axes[1, 0].set_ylabel('Cumulative Error (m)')
            axes[1, 0].legend()
            axes[1, 0].grid(True, alpha=0.3)
        
        # SLAM Analysis
        if self.slam:
            poses = self.slam.poses
            
            # Plot 5: Trajectory smoothness analysis
            pose_changes = np.diff(poses, axis=0)
            smoothness = np.linalg.norm(pose_changes, axis=1)
            
            axes[1, 1].plot(smoothness)
            axes[1, 1].set_title('Trajectory Smoothness')
            axes[1, 1].set_xlabel('Pose Index')
            axes[1, 1].set_ylabel('Pose Change Magnitude')
            axes[1, 1].grid(True, alpha=0.3)
            
            # Plot 6: Loop closure analysis
            if self.slam.loop_closures:
                lc_distances = []
                for lc in self.slam.loop_closures:
                    start_pose = poses[lc['start_idx']]
                    end_pose = poses[lc['end_idx']]
                    distance = np.linalg.norm(start_pose[:2] - end_pose[:2])
                    lc_distances.append(distance)
                
                axes[1, 2].bar(range(len(lc_distances)), lc_distances)
                axes[1, 2].set_title('Loop Closure Distances')
                axes[1, 2].set_xlabel('Loop Closure Index')
                axes[1, 2].set_ylabel('Distance (m)')
                axes[1, 2].grid(True, alpha=0.3)
            else:
                axes[1, 2].text(0.5, 0.5, 'No Loop Closures', 
                               ha='center', va='center', transform=axes[1, 2].transAxes)
                axes[1, 2].set_title('Loop Closure Analysis')
        
        plt.tight_layout()
        plt.savefig('/Users/ali/Documents/roboex/comprehensive_error_analysis.png', 
                   dpi=300, bbox_inches='tight')
        # plt.show()
        
    def generate_all_visualizations(self):
        """Generate all enhanced visualizations"""
        print("Generating enhanced visualizations...")
        
        print("  - Interactive sensor analysis...")
        self.create_interactive_sensor_analysis()
        
        print("  - Interactive motion analysis...")
        self.create_interactive_motion_analysis()
        
        print("  - Interactive SLAM dashboard...")
        self.create_interactive_slam_analysis()
        
        print("  - Comprehensive error analysis...")
        self.create_comprehensive_error_analysis()
        
        print("Enhanced visualizations completed!")

def create_enhanced_visualizations(sensor_model, motion_model, slam):
    """Main function to create enhanced visualizations"""
    viz = EnhancedVisualizations(sensor_model, motion_model, slam)
    viz.generate_all_visualizations()
    return viz

if __name__ == "__main__":
    # This would be called from main.py with the models
    print("Enhanced visualizations module ready!")
