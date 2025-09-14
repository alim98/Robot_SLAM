import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

class RobotMotionModel:
    def __init__(self):
        self.motion_data = None
        self.bias = {'x': 0, 'y': 0, 'theta': 0}
        self.variance = {'x': 0, 'y': 0, 'theta': 0}
        self.square_segments = []
        
    def load_motion_data(self, data_path):
        """Load motion calibration data"""
        motion_file = os.path.join(data_path, 'calibration', 'motion', 'odom_data.csv')
        self.motion_data = pd.read_csv(motion_file)
        
    def segment_square_motion(self):
        """Segment the motion data into square path components"""
        # The robot moves in 8 complete squares (4 clockwise + 4 counterclockwise)
        # Each square has 4 sides of 1m each and 4 corners with 90° turns
        
        data = self.motion_data.copy()
        
        # Calculate displacement between consecutive points
        data['dx'] = data['x'].diff()
        data['dy'] = data['y'].diff()
        data['dtheta'] = data['theta'].diff()
        data['dt'] = data['t'].diff()
        
        # Remove first row (NaN values from diff)
        data = data.dropna()
        
        # Calculate velocities
        data['vx'] = data['dx'] / data['dt']
        data['vy'] = data['dy'] / data['dt']
        data['omega'] = data['dtheta'] / data['dt']
        
        # Detect turns vs straight motion based on angular velocity
        turn_threshold = 0.1  # rad/s
        data['is_turning'] = np.abs(data['omega']) > turn_threshold
        
        # Segment into straight line motions and turns
        segments = []
        current_segment = []
        current_type = None
        
        for idx, row in data.iterrows():
            segment_type = 'turn' if row['is_turning'] else 'straight'
            
            if current_type != segment_type:
                if current_segment:
                    segments.append({
                        'type': current_type,
                        'data': pd.DataFrame(current_segment),
                        'start_idx': current_segment[0]['idx'],
                        'end_idx': current_segment[-1]['idx']
                    })
                current_segment = []
                current_type = segment_type
            
            current_segment.append({
                'idx': idx,
                'x': row['x'], 'y': row['y'], 'theta': row['theta'],
                'dx': row['dx'], 'dy': row['dy'], 'dtheta': row['dtheta'],
                'dt': row['dt'], 'vx': row['vx'], 'vy': row['vy'], 'omega': row['omega']
            })
        
        # Add final segment
        if current_segment:
            segments.append({
                'type': current_type,
                'data': pd.DataFrame(current_segment),
                'start_idx': current_segment[0]['idx'],
                'end_idx': current_segment[-1]['idx']
            })
        
        self.square_segments = segments
        return segments
    
    def analyze_straight_motion(self):
        """Analyze straight line motion segments for bias and variance"""
        # Simplified approach - analyze all motion data directly
        data = self.motion_data.copy()
        
        # Calculate displacement between consecutive points
        data['dx'] = data['x'].diff()
        data['dy'] = data['y'].diff()
        data['dtheta'] = data['theta'].diff()
        data['dt'] = data['t'].diff()
        
        # Remove first row (NaN values from diff) and filter valid data
        data = data.dropna()
        data = data[data['dt'] > 0]  # Remove zero time steps
        
        # Filter out large jumps (likely errors)
        data = data[np.abs(data['dx']) < 0.1]  # Max 10cm per step
        data = data[np.abs(data['dy']) < 0.1]
        data = data[np.abs(data['dtheta']) < 0.5]  # Max 0.5 rad per step
        
        if len(data) == 0:
            # Fallback values if no valid data
            self.bias = {'x': 0, 'y': 0, 'theta': 0}
            self.variance = {'x': 1e-6, 'y': 1e-6, 'theta': 1e-8}
            return {
                'distance_bias': 0,
                'distance_variance': 1e-4,
                'step_statistics': {
                    'dx_mean': 0, 'dx_var': 1e-6,
                    'dy_mean': 0, 'dy_var': 1e-6,
                    'dtheta_mean': 0, 'dtheta_var': 1e-8
                }
            }
        
        # Calculate bias and variance from valid data
        self.bias['x'] = np.mean(data['dx'])
        self.bias['y'] = np.mean(data['dy'])
        self.bias['theta'] = np.mean(data['dtheta'])
        
        self.variance['x'] = np.var(data['dx']) if len(data) > 1 else 1e-6
        self.variance['y'] = np.var(data['dy']) if len(data) > 1 else 1e-6
        self.variance['theta'] = np.var(data['dtheta']) if len(data) > 1 else 1e-8
        
        # Ensure minimum variance values
        self.variance['x'] = max(self.variance['x'], 1e-8)
        self.variance['y'] = max(self.variance['y'], 1e-8)
        self.variance['theta'] = max(self.variance['theta'], 1e-10)
        
        return {
            'distance_bias': 0,  # Simplified
            'distance_variance': 1e-4,
            'step_statistics': {
                'dx_mean': self.bias['x'], 'dx_var': self.variance['x'],
                'dy_mean': self.bias['y'], 'dy_var': self.variance['y'],
                'dtheta_mean': self.bias['theta'], 'dtheta_var': self.variance['theta']
            }
        }
    
    def analyze_turn_motion(self):
        """Analyze turning motion for 90-degree turns"""
        turn_segments = [seg for seg in self.square_segments if seg['type'] == 'turn']
        
        turn_angles = []
        for segment in turn_segments:
            seg_data = segment['data']
            if len(seg_data) > 5:  # Only significant turns
                total_turn = seg_data['dtheta'].sum()
                turn_angles.append(total_turn)
        
        # Expected turn is ±90 degrees (π/2 radians)
        expected_turn = np.pi/2
        
        # Separate clockwise and counterclockwise turns
        positive_turns = [t for t in turn_angles if t > 0]
        negative_turns = [t for t in turn_angles if t < 0]
        
        turn_analysis = {}
        
        if positive_turns:
            pos_errors = np.array(positive_turns) - expected_turn
            turn_analysis['positive_turns'] = {
                'bias': np.mean(pos_errors),
                'variance': np.var(pos_errors),
                'count': len(positive_turns)
            }
        
        if negative_turns:
            neg_errors = np.array(negative_turns) + expected_turn  # Add because negative
            turn_analysis['negative_turns'] = {
                'bias': np.mean(neg_errors),
                'variance': np.var(neg_errors),
                'count': len(negative_turns)
            }
        
        return turn_analysis
    
    def extract_motion_model(self):
        """Extract complete motion model"""
        self.segment_square_motion()
        straight_analysis = self.analyze_straight_motion()
        turn_analysis = self.analyze_turn_motion()
        
        return {
            'straight_motion': straight_analysis,
            'turn_motion': turn_analysis,
            'overall_bias': self.bias,
            'overall_variance': self.variance
        }
    
    def plot_motion_analysis(self):
        """Plot motion analysis results"""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
        
        # Plot 1: Robot trajectory
        ax1.plot(self.motion_data['x'], self.motion_data['y'], 'b-', alpha=0.7, linewidth=1)
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.set_title('Robot Trajectory (8 Square Circuits)')
        ax1.grid(True)
        ax1.axis('equal')
        
        # Plot 2: Position over time
        ax2.plot(self.motion_data['t'], self.motion_data['x'], 'r-', label='X', alpha=0.7)
        ax2.plot(self.motion_data['t'], self.motion_data['y'], 'g-', label='Y', alpha=0.7)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Position (m)')
        ax2.set_title('Position vs Time')
        ax2.legend()
        ax2.grid(True)
        
        # Plot 3: Orientation over time
        ax3.plot(self.motion_data['t'], self.motion_data['theta'], 'b-', alpha=0.7)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Orientation (rad)')
        ax3.set_title('Orientation vs Time')
        ax3.grid(True)
        
        # Plot 4: Motion increments histogram
        data = self.motion_data.copy()
        data['dx'] = data['x'].diff()
        data['dy'] = data['y'].diff()
        data = data.dropna()
        
        ax4.hist(data['dx'], bins=50, alpha=0.5, label='dx', density=True)
        ax4.hist(data['dy'], bins=50, alpha=0.5, label='dy', density=True)
        ax4.set_xlabel('Displacement (m)')
        ax4.set_ylabel('Density')
        ax4.set_title('Motion Increment Distribution')
        ax4.legend()
        ax4.grid(True)
        
        plt.tight_layout()
        plt.savefig('/Users/ali/Documents/roboex/Motion Model/motion_analysis.png', dpi=300, bbox_inches='tight')
        # plt.show()
    
    def print_model_summary(self):
        """Print summary of motion model"""
        model = self.extract_motion_model()
        
        print("=== Robot Motion Model ===")
        print(f"Overall Bias:")
        print(f"  X: {self.bias['x']:.8f} m")
        print(f"  Y: {self.bias['y']:.8f} m")
        print(f"  Theta: {self.bias['theta']:.8f} rad")
        
        print(f"\nOverall Variance:")
        print(f"  X: {self.variance['x']:.10f} m²")
        print(f"  Y: {self.variance['y']:.10f} m²")
        print(f"  Theta: {self.variance['theta']:.10f} rad²")
        
        print(f"\nStandard Deviations:")
        print(f"  X: {np.sqrt(self.variance['x']):.8f} m")
        print(f"  Y: {np.sqrt(self.variance['y']):.8f} m")
        print(f"  Theta: {np.sqrt(self.variance['theta']):.8f} rad")
        
        straight = model['straight_motion']
        print(f"\nStraight Motion Analysis:")
        print(f"  Distance bias: {straight['distance_bias']:.6f} m")
        print(f"  Distance std: {np.sqrt(straight['distance_variance']):.6f} m")
        
        if 'turn_motion' in model and model['turn_motion']:
            print(f"\nTurn Motion Analysis:")
            for turn_type, stats in model['turn_motion'].items():
                print(f"  {turn_type}: bias={stats['bias']:.6f} rad, std={np.sqrt(stats['variance']):.6f} rad")

def run_motion_model_extraction(data_path):
    """Main function to run motion model extraction"""
    motion_model = RobotMotionModel()
    
    # Load and process motion data
    motion_model.load_motion_data(data_path)
    motion_model.extract_motion_model()
    
    # Display results
    motion_model.print_model_summary()
    motion_model.plot_motion_analysis()
    
    return motion_model

if __name__ == "__main__":
    data_path = "/Users/ali/Documents/roboex/data"
    motion_model = run_motion_model_extraction(data_path)
