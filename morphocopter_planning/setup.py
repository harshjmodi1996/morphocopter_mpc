from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'morphocopter_planning'

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
    description='This package is responsible for creating feasible collision free path for a folding drone named morphocopter',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'OMPL_based_planner = morphocopter_planning.OMPL_based_planner:main',
        'RRT_waypoints = morphocopter_planning.RRT_waypoints:main',
        'RRTStar_waypoints = morphocopter_planning.RRTStar_waypoints:main',
        'subscribe_RRT_logging = morphocopter_planning.subscribe_RRT_logging:main',
        'BITMPC_setpoint_reader = morphocopter_planning.BITMPC_setpoint_reader:main',
        'lidar_scan_trial = morphocopter_planning.lidar_scan_trial:main',
        'acados_mpc = morphocopter_planning.acados_mpc:main',
        'acados_mpc2 = morphocopter_planning.acados_mpc2:main',
        'acados_mpc3 = morphocopter_planning.acados_mpc3:main',
        'mpc_trajectory_to_setpoint = morphocopter_planning.mpc_trajectory_to_setpoint:main',
        'lidar_data_processor = morphocopter_planning.lidar_data_processor:main',
        'segment_visualizer_rviz = morphocopter_planning.segment_visualizer_rviz:main',
        'lidar_data_forwarder = morphocopter_planning.lidar_data_forwarder:main',
        'publish_map_for_rviz = morphocopter_planning.publish_map_for_rviz:main',
        ],
    },
)
