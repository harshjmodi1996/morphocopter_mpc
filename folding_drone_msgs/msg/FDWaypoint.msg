# Waypoint message for folding drone

uint64 timestamp		# time since system start (microseconds)

geometry_msgs/PoseArray positions 	# desired positions at each waypoint, only provide position, orientation will be ignored
float32[3] initial_velocities 	    # desired initial velocities
float32[3] final_velocities         # desired final velocities
float32[] yaws  					# desired yaw in radians (FRD frame with joint angle consideration) at each waypoint
float32[] joint_angles  			# upper arm joint angle in radians at each waypoint
float32[] times 					# desired time in seconds to reach the waypoints from current location