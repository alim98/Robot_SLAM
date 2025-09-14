#!/usr/bin/env python3
"""
Main executable for RoboEx SLAM project
Runs sensor model extraction, motion model extraction, and Graph Optimization SLAM

Requirements: numpy, pandas, matplotlib, scipy
Compatible with Google Colab and MATLAB2020b (via Python engine)
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Add subdirectories to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'Sensor Model'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'Motion Model'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'SLAM'))

# Import our modules
from sensor_model import LidarSensorModel, run_sensor_model_extraction
from motion_model import RobotMotionModel, run_motion_model_extraction
from graph_optimization_slam import GraphOptimizationSLAM, run_graph_optimization_slam

def main():
    """Main function that runs the complete SLAM pipeline"""
    
    # Set data path
    data_path = os.path.join(os.path.dirname(__file__), 'data')
    
    print("="*60)
    print("RoboEx SLAM Project - Complete Pipeline")
    print("="*60)
    
    # Check if data directory exists
    if not os.path.exists(data_path):
        print(f"Error: Data directory not found at {data_path}")
        print("Please ensure the 'data' folder is in the same directory as main.py")
        return
    
    # Phase 1: Sensor Model Extraction
    print("\n" + "="*40)
    print("PHASE 1: LIDAR SENSOR MODEL EXTRACTION")
    print("="*40)
    
    try:
        sensor_model = run_sensor_model_extraction(data_path)
        print("✓ Sensor model extraction completed successfully")
    except Exception as e:
        print(f"✗ Error in sensor model extraction: {e}")
        sensor_model = None
    
    # Phase 2: Motion Model Extraction
    print("\n" + "="*40)
    print("PHASE 2: ROBOT MOTION MODEL EXTRACTION")
    print("="*40)
    
    try:
        motion_model = run_motion_model_extraction(data_path)
        print("✓ Motion model extraction completed successfully")
    except Exception as e:
        print(f"✗ Error in motion model extraction: {e}")
        motion_model = None
    
    # Phase 3: Graph Optimization SLAM
    print("\n" + "="*40)
    print("PHASE 3: GRAPH OPTIMIZATION SLAM")
    print("="*40)
    
    try:
        slam = run_graph_optimization_slam(data_path, sensor_model, motion_model)
        print("✓ SLAM completed successfully")
    except Exception as e:
        print(f"✗ Error in SLAM: {e}")
        slam = None
    
    # Phase 4: Enhanced Visualizations
    print("\n" + "="*40)
    print("PHASE 4: ENHANCED VISUALIZATIONS")
    print("="*40)
    
    try:
        from enhanced_visualizations import EnhancedVisualizations
        enhanced_viz = EnhancedVisualizations(sensor_model, motion_model, slam)
        enhanced_viz.generate_all_visualizations()
        print("✓ Enhanced visualizations completed successfully")
    except Exception as e:
        print(f"✗ Error in enhanced visualizations: {e}")
    
    # Phase 5: 3D Visualizations
    print("\n" + "="*40)
    print("PHASE 5: 3D VISUALIZATIONS")
    print("="*40)
    
    try:
        from enhanced_visualizations_3d import Enhanced3DVisualizations
        enhanced_3d_viz = Enhanced3DVisualizations(sensor_model, motion_model, slam)
        enhanced_3d_viz.generate_all_3d_visualizations()
        print("✓ 3D visualizations completed successfully")
    except Exception as e:
        print(f"✗ Error in 3D visualizations: {e}")
    
    # Summary
    print("\n" + "="*60)
    print("EXECUTION SUMMARY")
    print("="*60)
    
    if sensor_model:
        stats = sensor_model.get_overall_statistics()
        print(f"✓ Sensor Model: bias={stats['overall_bias']:.6f}m, var={stats['overall_variance']:.8f}m²")
    else:
        print("✗ Sensor Model: Failed")
    
    if motion_model:
        print(f"✓ Motion Model: x_var={motion_model.variance['x']:.10f}m², y_var={motion_model.variance['y']:.10f}m², θ_var={motion_model.variance['theta']:.10f}rad²")
    else:
        print("✗ Motion Model: Failed")
    
    if slam:
        print(f"✓ SLAM: {len(slam.poses)} poses optimized, {len(slam.loop_closures)} loop closures")
        if slam.occupancy_grid is not None:
            print(f"✓ Map: {slam.grid_size[0]}×{slam.grid_size[1]} grid at {slam.grid_resolution}m resolution")
    else:
        print("✗ SLAM: Failed")
    
    print("\n" + "="*60)
    print("All results saved to respective subdirectories:")
    print("Standard Visualizations:")
    print("- Sensor Model/lidar_calibration.png")
    print("- Motion Model/motion_analysis.png") 
    print("- SLAM/map_evolution.png")
    print("- SLAM/final_results.png")
    print("- comprehensive_error_analysis.png")
    print("\nInteractive 2D Visualizations:")
    print("- Sensor Model/interactive_sensor_analysis.html")
    print("- Motion Model/interactive_motion_3d.html")
    print("- Motion Model/motion_statistics_dashboard.html")
    print("- SLAM/interactive_slam_dashboard.html")
    print("\nInteractive 3D Visualizations:")
    print("- Sensor Model/sensor_3d_analysis.html")
    print("- Motion Model/motion_3d_enhanced.html")
    print("- SLAM/slam_3d_trajectory.html")
    print("- SLAM/occupancy_grid_3d.html")
    print("- error_analysis_3d.html")
    print("="*60)

def check_dependencies():
    """Check if required dependencies are available"""
    required_packages = ['numpy', 'pandas', 'matplotlib', 'plotly', 'seaborn']
    missing_packages = []
    
    for package in required_packages:
        try:
            __import__(package)
        except ImportError:
            missing_packages.append(package)
    
    if missing_packages:
        print("Missing required packages:")
        for pkg in missing_packages:
            print(f"  - {pkg}")
        print("\nInstall with: pip install " + " ".join(missing_packages))
        return False
    
    return True

def setup_matplotlib():
    """Setup matplotlib for different environments"""
    # Configure matplotlib for different backends
    import matplotlib
    
    # Try to use interactive backend if available
    try:
        matplotlib.use('TkAgg')
    except:
        try:
            matplotlib.use('Qt5Agg')
        except:
            # Fall back to non-interactive backend for Colab/headless
            matplotlib.use('Agg')
    
    # Set style
    plt.style.use('default')
    plt.rcParams['figure.figsize'] = (12, 8)
    plt.rcParams['font.size'] = 10

if __name__ == "__main__":
    # Check dependencies
    if not check_dependencies():
        sys.exit(1)
    
    # Setup matplotlib
    setup_matplotlib()
    
    # Run main pipeline
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nExecution interrupted by user")
    except Exception as e:
        print(f"\n\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
