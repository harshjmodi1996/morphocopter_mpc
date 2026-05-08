import csv
import numpy as np

def interpolate_path_poses(path_poses,interpolation_interval):
    interpolated_poses = []
    for i in range(1, len(path_poses)):
        p0 = path_poses[i - 1][0:3]
        p1 = path_poses[i][0:3]
        yaw0 = path_poses[i - 1][3]
        yaw1 = path_poses[i][3]
        direction_vector = np.array([p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]])
        line_length = np.linalg.norm(direction_vector)
        direction_vector = direction_vector / line_length
        num_points = int(line_length / interpolation_interval)  # Number of points to sample along the line
        
        for j in range(1, num_points + 1):
            point_on_line = np.array([p0[0], p0[1], p0[2]]) + direction_vector * j * interpolation_interval
            interpolated_yaw = yaw0 + (yaw1 - yaw0) * (j / num_points)
            interpolated_yaw = interpolated_yaw % (2 * np.pi)  # Normalize yaw to [0, 2π]

            interpolated_pose = [point_on_line[0], point_on_line[1], point_on_line[2],interpolated_yaw]
            interpolated_poses.append(interpolated_pose)

    return interpolated_poses


csvreader = csv.reader(open('morphocopter_waypoints_custom.csv', 'r'))
path_poses = []
for row in csvreader:
    try:
        path_poses.append([float(i) for i in row])
    except ValueError:
        continue

interpolated_poses = interpolate_path_poses(path_poses,0.005)

csvwriter = csv.writer(open('morphocopter_waypoints.csv', 'w', newline=''))
i = 0
for pose in interpolated_poses:
    csvwriter.writerow(pose)
    i += 1

print(f"Interpolated waypoints written to morphocopter_waypoints.csv, total waypoints: {i}")




