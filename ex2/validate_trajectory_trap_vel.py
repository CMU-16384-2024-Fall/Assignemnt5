import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat

def validate_trajectory_trap_vel():
    # Load saved data
    data = loadmat('ex_02_ground_truth.mat')
    waypoints = data['waypoints'][0]
    times = data['times'][0]
    traj = data['traj'][0]
    duty_cycles = data['duty_cycles'][0]

    frequency = 100  # control frequency

    plt.figure(figsize=(12, 4 * len(waypoints)))
    num_traj = len(waypoints)
    for i in range(num_traj):
        my_traj = trajectory_trap_vel(waypoints[i], times[i], frequency, duty_cycles[i][0])
        traj_time = np.linspace(times[i][0, 0], times[i][0, -1], my_traj.shape[1])
        
        # Plot actual data
        plt.subplot(num_traj, 1, i + 1)
        plt.plot(traj_time, my_traj.T, 'k-', linewidth=1)
        plt.title(f'Trapezoidal Velocity Joint Trajectory - Sample {i + 1}')
        plt.xlabel('t [s]')
        plt.ylabel('θ [rad]')
        
        # Plot additional ground truth
        plt.plot(traj_time, traj[i].T, 'g--', linewidth=1)
        plt.xlim(times[i][0, 0], times[i][0, -1])
        plt.legend(['Your Trajectory', 'Correct Trajectory'], loc='upper right')

    plt.tight_layout()
    plt.show()

def trajectory_trap_vel(waypoints, times, frequency, duty_cycle):
    """
    Returns a matrix of joint angles, where each column represents a single
    timestamp. These joint angles form trapezoidal velocity trajectory segments,
    hitting waypoints[:, i] at times[i].

    Args:
    waypoints (np.array): Matrix of waypoints; each column represents a single
                          waypoint in joint space, and each row represents a particular joint.
    times (np.array): Row vector that indicates the time each of the waypoints should
                      be reached. The number of columns should equal the number of waypoints,
                      and should be monotonically increasing.
    frequency (float): The control frequency at which this trajectory should be played,
                       and therefore the number of columns per second of playback.
    duty_cycle (float): The duty cycle for the trapezoidal velocity profile.

    Returns:
    np.array: Matrix of joint angles forming the trajectory.
    """
    # Number of joints
    num_joints = waypoints.shape[0]
    # Number of waypoints
    num_waypoints = waypoints.shape[1]
    # Number of segments between waypoints
    num_segments = num_waypoints - 1

    if times.shape != (1, num_waypoints):
        raise ValueError('Size of times vector is incorrect!')

    if num_waypoints < 2:
        raise ValueError('Insufficient number of waypoints.')

    if not isinstance(frequency, (int, float)) or frequency < 5:
        raise ValueError('Invalid control frequency (must be at least 5Hz)')

    if duty_cycle < 0 or duty_cycle > 0.5:
        raise ValueError('Invalid duty cycle!')

    # Calculate number of points per segment
    num_points_per_segment = []
    for segment in range(num_segments):
        dt = times[0, segment + 1] - times[0, segment]
        num_points_per_segment.append(int(dt * frequency))

    # Pre-allocate trajectory matrix
    trajectory = np.zeros((num_joints, int(np.sum(num_points_per_segment))))

    # Fill in trajectory segment-by-segment
    segment_start_point = 0
    for segment in range(num_segments):
        points_in_segment = num_points_per_segment[segment]
        segment_end_point = segment_start_point + points_in_segment

        num_ramp_points = int(duty_cycle * points_in_segment)
        ramp_time = (times[0, segment + 1] - times[0, segment]) * duty_cycle

        # --------------- BEGIN STUDENT SECTION ----------------------------------
        # TODO: Calculate the maximum velocity for this segment
        vm = np.zeros(num_joints)  # Replace this line with your calculation

        # TODO: Fill in the points for this segment of the trajectory
        # You need to implement the trapezoidal velocity profile here
        # Hint: Use three phases: ramp up, constant velocity, and ramp down

        # Example structure (you need to fill in the correct calculations):
        for joint in range(num_joints):
            q0 = waypoints[joint, segment]
            qf = waypoints[joint, segment + 1]
            v_max = vm[joint]

            for i in range(points_in_segment):
                t = i / frequency
                if i < num_ramp_points:
                    # TODO: Implement ramp up phase
                    q = 0  # Replace with correct calculation
                elif i >= points_in_segment - num_ramp_points:
                    # TODO: Implement ramp down phase
                    q = 0  # Replace with correct calculation
                else:
                    # TODO: Implement constant velocity phase
                    q = 0  # Replace with correct calculation

                trajectory[joint, segment_start_point + i] = q

        # --------------- END STUDENT SECTION ------------------------------------

        segment_start_point += points_in_segment

    return trajectory

# Call the validation function
if __name__ == "__main__":
    validate_trajectory_trap_vel()