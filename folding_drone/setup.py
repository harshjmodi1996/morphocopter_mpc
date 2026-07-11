from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'folding_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anonymous',
    maintainer_email='anonymous@anonymous.edu',
    description='A package to control a folding drone which has ability to rotate upper diagonal arm using UXRCE_DDS communication',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'offboard_publisher = folding_drone.offboard_publisher:main',
        'outerloop_error_calculations = folding_drone.outerloop_error_calculations:main',
	'attitude_control = folding_drone.attitude_control:main',
        'nonlinear_controller = folding_drone.nonlinear_controller:main',
        'thrust_measurement = folding_drone.thrust_measurement:main',
        'waypoints_to_setpoints = folding_drone.waypoints_to_setpoints:main',
        'waypoints_trial = folding_drone.waypoints_trial:main',
        'vehicle_local_position_update = folding_drone.vehicle_local_position_update:main',
        'joint_angle_publisher = folding_drone.joint_angle_publisher:main',
        'setpoint_reader = folding_drone.setpoint_reader:main',
        'smooth_waypoints_from_fixed_setpoint = folding_drone.smooth_waypoints_from_fixed_setpoint:main',
        'waypoints_1_obstacle = folding_drone.waypoints_1_obstacle:main',
        'waypoints_multi_obstacles = folding_drone.waypoints_multi_obstacles:main',
        'waypoints_diamond = folding_drone.waypoints_diamond:main',
        'waypoints_shape_8 = folding_drone.waypoints_shape_8:main',
        ],
    },
)
