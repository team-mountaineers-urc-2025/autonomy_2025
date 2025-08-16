from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomy_2025'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jdb3',
    maintainer_email='jalen.beeman@gmail.com',
    description='TODO: Package description',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_p2p = autonomy_2025.static_p2p_vel:main',
            'local_path_planner = autonomy_2025.local_path_planner:main',
            'brian = autonomy_2025.brian_node:main',
            'pose_localizer = autonomy_2025.position_localizer:main',
            'pid_planner_node = autonomy_2025.new_pid_planner:main',
            'control_mux = autonomy_2025.move_controller_mux:main',
            'led_interface = autonomy_2025.led_interface:main',
            'velocity_mux = autonomy_2025.velocity_mux:main',
            'waypoint_manager = autonomy_2025.waypoint_manager:main',
            'camera_info = autonomy_2025.camera_info:main'
        ],
    },
)
