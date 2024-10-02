# validate_trajectory_const_vel.py

import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat
import sys
import os

# Import the trajectory_const_vel function from the module
from trajectory_const_vel import trajectory_const_vel

def validate_trajectory_const_vel(mat_file='ex_01_ground_truth.mat', frequency=100):
    """
    Validates the trajectory_const_vel function by comparing generated trajectories
    against ground truth data loaded from a .mat file.

    Parameters:
    - mat_file: str
        Path to the .mat file containing 'waypoints', 'times', and 'traj'.
    - frequency: float
        Control frequency in Hz used to generate trajectories.
    """
    # Check if the .mat file exists
    if not os.path.isfile(mat_file):
        print(f"Error: The file '{mat_file}' was not found in the current directory.")
        sys.exit(1)
    
    # Load saved data
    try:
        data = loadmat(mat_file)
    except Exception as e:
        print(f"Error loading '{mat_file}': {e}")
        sys.exit(1)
    
    # Extract data
    # Assuming 'waypoints', 'times', and 'traj' are stored as cell arrays in MATLAB
    waypoints = data.get('waypoints')
    times = data.get('times')
    traj_ground_truth = data.get('traj')
    
    if waypoints is None or times is None or traj_ground_truth is None:
        print("Error: The .mat file must contain 'waypoints', 'times', and 'traj' variables.")
        sys.exit(1)
    
    # Convert MATLAB cell arrays to Python lists
    def extract_cell_array(cell_array):
        if isinstance(cell_array, np.ndarray) and cell_array.dtype == object:
            return [np.squeeze(item) for item in cell_array.flatten()]
        else:
            return [cell_array]
    
    waypoints = extract_cell_array(waypoints)
    times = extract_cell_array(times)
    traj_ground_truth = extract_cell_array(traj_ground_truth)
    
    num_traj = len(waypoints)
    if not (len(times) == num_traj and len(traj_ground_truth) == num_traj):
        print("Error: The number of waypoints, times, and traj entries must be equal.")
        sys.exit(1)
    
    # Create a figure with subplots for each trajectory
    for i in range(num_traj):
        current_waypoints = waypoints[i]
        current_times = times[i].flatten()
        current_gt_traj = traj_ground_truth[i]
        
        # Validate shapes
        if current_waypoints.ndim != 2:
            print(f"Error: Waypoints for sample {i+1} must be a 2D array.")
            continue
        if current_gt_traj.ndim != 2:
            print(f"Error: Ground truth trajectory for sample {i+1} must be a 2D array.")
            continue
        
        num_joints, num_waypoints = current_waypoints.shape
        gt_num_joints, gt_num_points = current_gt_traj.shape
        
        if num_joints != gt_num_joints:
            print(f"Error: Number of joints mismatch in sample {i+1}.")
            continue
        
        # Generate trajectory using the translated function
        try:
            my_traj = trajectory_const_vel(current_waypoints, current_times, frequency)
        except Exception as e:
            print(f"Error generating trajectory for sample {i+1}: {e}")
            continue
        
        # Generate time vector for generated trajectory
        my_traj_time = np.linspace(current_times[0], current_times[-1], my_traj.shape[1])
        
        # Generate time vector for ground truth trajectory
        gt_traj_time = np.linspace(current_times[0], current_times[-1], current_gt_traj.shape[1])
        
        # Plotting
        plt.figure(figsize=(10, 4 * num_joints))
        for joint in range(num_joints):
            plt.subplot(num_joints, 1, joint + 1)
            plt.plot(my_traj_time, my_traj[joint, :], 'k-', label='Your Trajectory')
            plt.plot(gt_traj_time, current_gt_traj[joint, :], 'g--', label='Ground Truth' if joint == 0 else "")
            plt.title(f'Sample {i + 1} - Joint {joint + 1}')
            plt.xlabel('Time [s]')
            plt.ylabel('θ [rad]')
            plt.legend(loc='upper right')
            plt.grid(True)
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    # Execute the validation
    validate_trajectory_const_vel()
