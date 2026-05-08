# Setpoint message for folding drone

uint64 timestamp		# time since system start (microseconds)

geometry_msgs/Point position
geometry_msgs/Point velocity
geometry_msgs/Point acceleration

float32 yaw  # desired yaw in radians (FRD frame with joint angle consideration)
float32 joint_angle  # upper arm joint angle in radians