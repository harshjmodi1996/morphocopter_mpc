import os
from pathlib import Path
import numpy as np


def getTrack(track):
    track_file = os.path.join(str(Path(__file__).parent), "tracks/", track)
    array = np.loadtxt(track_file, delimiter=',', skiprows=1)


    ################### TEMP FOR DATA COLLECTION OF MEEN 689 ####################
    first_waypoint = array[0].copy()
    first_waypoint[2] = 0.3  # initiate z at 0.3
    array = np.insert(array, 0, first_waypoint, axis=0)
    ##############################################################################

    unique_indices = []
    xref = []
    yref = []
    zref = []

    for row in array:
        # converting ENU coordinates to NED coordinates
        xref.append(row[1])
        yref.append(row[0])
        zref.append(-row[2]-0.0) 

    xref = np.array(xref)
    yref = np.array(yref)
    zref = np.array(zref)
    reference_waypoints = np.column_stack((xref, yref, zref))

    return reference_waypoints

def upscale_waypoints_based_on_distance(reference_waypoints, distance_threshold, log_csv=False, csv_filename="upscaled_waypoints.csv", smoothing_window_size=5):
    """
    Upscale waypoints based on a distance threshold and smooth them.
    Args:
        reference_waypoints (np.ndarray): Original waypoints, shape (N, 3)
        distance_threshold (float): Distance threshold for upscaling waypoints
        log_csv (bool): Whether to save the upscaled waypoints to a CSV file
        csv_filename (str): The name of the CSV file to save to
        smoothing_window_size (int): The size of the window for moving average smoothing (must be odd)
    Returns:
        np.ndarray: Upscaled and smoothed waypoints, shape (M, 3)
    """
    upscaled_waypoints = [reference_waypoints[0]]
    for i in range(1, reference_waypoints.shape[0]):
        segment = reference_waypoints[i] - reference_waypoints[i - 1]
        segment_length = np.linalg.norm(segment)
        if segment_length > distance_threshold:
            num_new_points = int(np.ceil(segment_length / distance_threshold))
            for j in range(1, num_new_points + 1):
                new_point = reference_waypoints[i - 1] + (segment * j / num_new_points)
                upscaled_waypoints.append(new_point)
        else:
            upscaled_waypoints.append(reference_waypoints[i])
    
    upscaled_waypoints_np = np.array(upscaled_waypoints)

    # Apply smoothing (Moving Average)
    if smoothing_window_size > 1:
        window = np.ones(smoothing_window_size) / smoothing_window_size
        # Pad the array to handle edges properly (reflect mode works well for trajectories)
        pad_width = smoothing_window_size // 2
        
        smoothed_x = np.convolve(np.pad(upscaled_waypoints_np[:, 0], pad_width, mode='edge'), window, mode='valid')
        smoothed_y = np.convolve(np.pad(upscaled_waypoints_np[:, 1], pad_width, mode='edge'), window, mode='valid')
        smoothed_z = np.convolve(np.pad(upscaled_waypoints_np[:, 2], pad_width, mode='edge'), window, mode='valid')
        
        upscaled_waypoints_np = np.column_stack((smoothed_x, smoothed_y, smoothed_z))

    if log_csv:
        np.savetxt(csv_filename, upscaled_waypoints_np, delimiter=",", header="x,y,z", comments="")
        print(f"Upscaled waypoints saved to {csv_filename}")

    return upscaled_waypoints_np

def upscale_waypoints(reference_waypoints, upscaling_multiplier):
    """
    Linearly interpolate waypoints to increase the number of waypoints by upscaling_multiplier.
    Args:
        reference_waypoints (np.ndarray): Original waypoints, shape (N, 3)
        upscaling_multiplier (int): Factor by which to increase the number of waypoints
    Returns:
        np.ndarray: Upscaled waypoints, shape (N * upscaling_multiplier - (upscaling_multiplier - 1), 3)
    """
    if upscaling_multiplier <= 1:
        return reference_waypoints

    N = reference_waypoints.shape[0]
    # Original indices
    orig_idx = np.arange(N)
    # New indices for upscaled waypoints
    new_N = (N - 1) * upscaling_multiplier + 1
    new_idx = np.linspace(0, N - 1, new_N)
    # Interpolate each coordinate
    x_up = np.interp(new_idx, orig_idx, reference_waypoints[:, 0])
    y_up = np.interp(new_idx, orig_idx, reference_waypoints[:, 1])
    z_up = np.interp(new_idx, orig_idx, reference_waypoints[:, 2])
    upscaled_waypoints = np.column_stack((x_up, y_up, z_up))
    return upscaled_waypoints

def compute_average_distance(waypoints):
    """
    Compute the average distance between consecutive waypoints.
    Args:
        waypoints (np.ndarray): Waypoints, shape (N, 3)
    Returns:
        float: Average distance between consecutive waypoints
    """
    if waypoints.shape[0] < 2:
        return 0.0

    distances = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    average_distance = np.mean(distances)
    return average_distance