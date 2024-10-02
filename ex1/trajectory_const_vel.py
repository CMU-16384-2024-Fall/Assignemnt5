import numpy as np

def trajectory_const_vel(waypoints, times, frequency):
    """
    trajectory_const_vel

    Returns a matrix of joint angles, where each column represents a single
    timestamp. These joint angles form constant velocity trajectory segments,
    hitting waypoints[:, i] at times[i].

    Parameters:
    - waypoints: np.ndarray
        Matrix of waypoints; each column represents a single waypoint in joint space,
        and each row represents a particular joint.
    - times: np.ndarray
        1D array that indicates the time each of the waypoints should be reached.
        The number of elements should equal the number of waypoints and should be
        monotonically increasing. The first element is typically 0.
    - frequency: float
        Control frequency in Hz at which this trajectory should be played.
        Must be at least 5Hz.

    Returns:
    - trajectory: np.ndarray
        Matrix of joint angles with shape (num_joints, total_points).
        Each column represents joint angles at a specific timestamp.
    """
    # Number of joints:
    num_joints, num_waypoints = waypoints.shape
    # Number of segments between waypoints:
    num_segments = num_waypoints - 1

    # Validate the size of the 'times' vector.
    if times.shape != (num_waypoints,):
        raise ValueError("Size of times vector is incorrect!")

    #Ensure there are at least two waypoints.
    if num_waypoints < 2:
        raise ValueError("Insufficient number of waypoints.")

    # Validate the 'frequency' input.
    if not isinstance(frequency, (int, float)) or frequency < 5:
        raise ValueError("Invalid control frequency (must be at least 5Hz)")

    ## First, calculate the number of points to generate for each segment based on the frequency.
    num_points_per_segment = np.zeros(num_segments, dtype=int)
    for segment in range(num_segments):
        dt = times[segment + 1] - times[segment]
        num_points_per_segment[segment] = int(dt * frequency)

    ## Pre-allocate the trajectory matrix with the total number of points.
    trajectory = np.zeros((num_joints, np.sum(num_points_per_segment)))

    # Initialize the starting index for the first segment
    segment_start_point = 0
    for segment in range(num_segments):
        # Number of points in the current segment
        points_in_segment = num_points_per_segment[segment]

        # Calculate the ending index for the current segment
        segment_end_point = segment_start_point + points_in_segment

        # --------------- BEGIN STUDENT SECTION ----------------------------------
        # Fill in the points in this segment of the trajectory.
        # The points should transition linearly from waypoints[:, segment] to waypoints[:, segment + 1]
        #
        # HINT:
        # - Use numpy.linspace to generate linearly spaced values for each joint.
        # - Assign these values to the appropriate slice in the 'trajectory' matrix.
        
        pass  # Replace this line with your implementation
        # --------------- END STUDENT SECTION ------------------------------------

        # Update the starting index for the next segment
        segment_start_point = segment_end_point

    # TODO: Optionally, append the final waypoint to the trajectory to ensure it reaches the exact end position.
    # HINT: You can use numpy.hstack or similar functions to add the last waypoint.

    return trajectory
