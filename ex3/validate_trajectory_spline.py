import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat
from scipy.interpolate import CubicSpline

def validate_trajectory_spline():
    # Load saved data
    data = loadmat('ex_03_ground_truth.mat')
    waypoints = data['waypoints'][0]  # List of arrays
    times = data['times'][0]  # List of arrays

    frequency = 100  # control frequency

    plt.figure(figsize=(12, 4*len(waypoints)))
    num_traj = len(waypoints)
    for i in range(num_traj):
        # Extract waypoints and times for this trajectory
        traj_waypoints = waypoints[i]
        traj_times = times[i].flatten()

        my_traj = trajectory_spline(traj_waypoints, traj_times, frequency)
        
        # Calculate the number of points based on the loaded trajectory data
        num_points = data['traj'][0][i].shape[1]
        
        # Generate time points to match the number of points in the trajectory data
        traj_time = np.linspace(traj_times[0], traj_times[-1], num_points)
        
        # Ensure my_traj has the same number of points
        if my_traj.shape[1] != num_points:
            my_traj_interp = np.zeros_like(data['traj'][0][i])
            for j in range(my_traj.shape[0]):
                my_traj_interp[j] = np.interp(traj_time, np.linspace(traj_times[0], traj_times[-1], my_traj.shape[1]), my_traj[j])
            my_traj = my_traj_interp
        
        # Plot actual data
        plt.subplot(num_traj, 1, i+1)
        plt.plot(traj_time, my_traj.T, 'k-', linewidth=1, label='Your Trajectory')
        plt.title(f'Spline Joint Trajectory - Sample {i+1}')
        plt.xlabel('t [s]')
        plt.ylabel('θ [rad]')
        
        # Plot additional ground truth
        plt.plot(traj_time, data['traj'][0][i].T, 'g--', linewidth=1, label='Correct Trajectory')
        plt.xlim(traj_times[0], traj_times[-1])
        plt.legend(loc='upper right')

    plt.tight_layout()
    plt.show()

def trajectory_spline(waypoints, times, frequency):
    """
    Computes a spline trajectory that passes through the given waypoints at the specified times.

    Args:
    waypoints (np.array): A 2D array where each column is a waypoint and each row is a joint.
    times (np.array): A 1D array of times corresponding to each waypoint.
    frequency (float): The desired frequency of the output trajectory.

    Returns:
    np.array: A 2D array where each column is a point on the trajectory and each row is a joint.
    """
    # Ensure inputs are numpy arrays
    waypoints = np.array(waypoints)
    times = np.array(times).flatten()

    # Number of joints and waypoints
    num_joints, num_waypoints = waypoints.shape

    if times.shape[0] != num_waypoints:
        raise ValueError('Size of times vector is incorrect!')

    if num_waypoints < 2:
        raise ValueError('Insufficient number of waypoints.')

    if not isinstance(frequency, (int, float)) or frequency < 5:
        raise ValueError('Invalid control frequency (must be at least 5Hz)')

    # Calculate total number of points
    total_time = times[-1] - times[0]
    num_points = int(total_time * frequency) + 1

    # Generate time points for the trajectory
    t = np.linspace(times[0], times[-1], num_points)

    # Initialize trajectory
    trajectory = np.zeros((num_joints, num_points))

    # TODO: Student implementation starts here
    
    # Hint: You may want to use scipy.interpolate.CubicSpline to create the spline.
    # Remember to handle each joint separately and ensure zero velocity at endpoints.
    
    # Example structure (you need to fill in the details):
    # for joint in range(num_joints):
    #     # Create spline for this joint
    #     # ...
    #     
    #     # Evaluate spline
    #     # trajectory[joint, :] = ...

    # TODO: Student implementation ends here

    return trajectory

if __name__ == "__main__":
    validate_trajectory_spline()