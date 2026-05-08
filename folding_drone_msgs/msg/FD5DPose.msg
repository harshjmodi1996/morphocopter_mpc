# message for publishing 3D position, quaternion, and joint angle from path planner for folding drone

uint64 timestamp		# time since system start (microseconds)

geometry_msgs/Point position
geometry_msgs/Quaternion orientation

float32 joint_angle  # upper arm joint angle in radians

float32 obstacle_distance
