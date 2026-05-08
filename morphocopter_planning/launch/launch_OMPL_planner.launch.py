from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the full path to your Python script
    # This assumes the script is installed in the lib directory next to other executables
    # script_path = os.path.join(
    #     os.path.expanduser('~'),
    #     'carl_ws/src/carl_ws_src/morphocopter_planning/scripts/randomize_sdf_and_generate_urdf.py'
    # )
    # # Create an action to execute the Python script
    # run_conversion_script = ExecuteProcess(
    #     cmd=['python3', script_path],
    #     output='screen'
    # )
    return LaunchDescription([

        # # Run the script first
        # run_conversion_script,

        Node(
            package='morphocopter_planning',
            executable='OMPL_based_planner',
        ),
    	Node(
            package='morphocopter_planning',
            executable='subscribe_RRT_logging',
        ),
    ])
