import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

class LidarSensorModel:
    def __init__(self):
        self.bias = {}
        self.variance = {}
        self.calibration_data = {}
        
    def load_calibration_data(self, data_path):
        """Load LiDAR calibration data from CSV files"""
        lidar_path = os.path.join(data_path, 'calibration', 'lidar')
        
        # Distance files available (in cm, convert to meters)
        distances = [5, 10, 15, 20, 25, 30, 37]
        
        for dist_cm in distances:
            file_path = os.path.join(lidar_path, f'distance_{dist_cm}.csv')
            if os.path.exists(file_path):
                df = pd.read_csv(file_path)
                true_distance = dist_cm / 100.0  # Convert cm to meters
                measured_distances = df['range'].values
                self.calibration_data[true_distance] = measured_distances
                
    def extract_sensor_model(self):
        """Extract bias and variance for each calibration distance"""
        for true_dist, measurements in self.calibration_data.items():
            # Calculate bias (systematic error)
            mean_measurement = np.mean(measurements)
            bias = mean_measurement - true_dist
            
            # Calculate variance (measurement noise)
            variance = np.var(measurements)
            
            self.bias[true_dist] = bias
            self.variance[true_dist] = variance
            
    def get_overall_statistics(self):
        """Calculate overall sensor model parameters"""
        all_biases = list(self.bias.values())
        all_variances = list(self.variance.values())
        
        overall_bias = np.mean(all_biases)
        overall_variance = np.mean(all_variances)
        
        return {
            'overall_bias': overall_bias,
            'overall_variance': overall_variance,
            'distance_specific_bias': self.bias,
            'distance_specific_variance': self.variance
        }
        
    def plot_calibration_results(self):
        """Plot calibration results showing bias and variance"""
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))
        
        # Plot 1: Measured vs True distances
        true_dists = list(self.calibration_data.keys())
        mean_measurements = [np.mean(self.calibration_data[d]) for d in true_dists]
        std_measurements = [np.std(self.calibration_data[d]) for d in true_dists]
        
        ax1.errorbar(true_dists, mean_measurements, yerr=std_measurements, 
                    fmt='o-', capsize=5, label='Measured')
        ax1.plot(true_dists, true_dists, 'r--', label='Perfect (y=x)')
        ax1.set_xlabel('True Distance (m)')
        ax1.set_ylabel('Measured Distance (m)')
        ax1.set_title('LiDAR Calibration: Measured vs True')
        ax1.legend()
        ax1.grid(True)
        
        # Plot 2: Bias vs Distance
        biases = [self.bias[d] for d in true_dists]
        ax2.plot(true_dists, biases, 'bo-')
        ax2.axhline(y=0, color='r', linestyle='--', alpha=0.7)
        ax2.set_xlabel('True Distance (m)')
        ax2.set_ylabel('Bias (m)')
        ax2.set_title('LiDAR Bias vs Distance')
        ax2.grid(True)
        
        # Plot 3: Variance vs Distance
        variances = [self.variance[d] for d in true_dists]
        ax3.plot(true_dists, variances, 'go-')
        ax3.set_xlabel('True Distance (m)')
        ax3.set_ylabel('Variance (m²)')
        ax3.set_title('LiDAR Variance vs Distance')
        ax3.grid(True)
        
        plt.tight_layout()
        plt.savefig('/Users/ali/Documents/roboex/Sensor Model/lidar_calibration.png', dpi=300, bbox_inches='tight')
        # plt.show()
        
    def correct_measurement(self, measurement, expected_distance=None):
        """Apply bias correction to a measurement"""
        if expected_distance and expected_distance in self.bias:
            return measurement - self.bias[expected_distance]
        else:
            # Use overall bias if specific distance not available
            overall_stats = self.get_overall_statistics()
            return measurement - overall_stats['overall_bias']
            
    def print_model_summary(self):
        """Print summary of extracted sensor model"""
        stats = self.get_overall_statistics()
        
        print("=== LiDAR Sensor Model ===")
        print(f"Overall Bias: {stats['overall_bias']:.6f} m")
        print(f"Overall Variance: {stats['overall_variance']:.8f} m²")
        print(f"Overall Standard Deviation: {np.sqrt(stats['overall_variance']):.6f} m")
        print("\nDistance-specific parameters:")
        
        for true_dist in sorted(stats['distance_specific_bias'].keys()):
            bias = stats['distance_specific_bias'][true_dist]
            variance = stats['distance_specific_variance'][true_dist]
            print(f"  {true_dist:.2f}m: bias={bias:.6f}m, variance={variance:.8f}m²")

def run_sensor_model_extraction(data_path):
    """Main function to run sensor model extraction"""
    sensor_model = LidarSensorModel()
    
    # Load and process calibration data
    sensor_model.load_calibration_data(data_path)
    sensor_model.extract_sensor_model()
    
    # Display results
    sensor_model.print_model_summary()
    sensor_model.plot_calibration_results()
    
    return sensor_model

if __name__ == "__main__":
    data_path = "/Users/ali/Documents/roboex/data"
    sensor_model = run_sensor_model_extraction(data_path)
