# message for publishing waypoint position along with distance of the nearest obstacle at various rays perpendicular to movement direction

uint64 timestamp		# time since system start (microseconds)

int32  index # index of the waypoint
geometry_msgs/Point waypoint  # Position of a waypoint
geometry_msgs/Vector3[] ray_directions   # unit vector in the direction of the ray test for obstacle distance [x,y,z]
float32[] obstacle_distances   # distance to the nearest obstacle in the given direction from given position
