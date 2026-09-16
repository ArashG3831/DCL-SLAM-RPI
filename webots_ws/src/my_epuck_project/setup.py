from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_epuck_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arash',
    maintainer_email='arash.ganjei92@gmail.com',
    description='Local Webots e-puck project package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'real_diffdrive_node = my_epuck_project.real_diffdrive_node:main',
            'd500_scan_fix = my_epuck_project.d500_scan_fix:main',
            'twist_stamper = my_epuck_project.twist_stamper:main',
            'map_exporter = my_epuck_project.map_exporter:main',
            'source_aware_map_fusion = my_epuck_project.source_aware_map_fusion:main',
            'unknown_pose_frontend = '
            'my_epuck_project.unknown_pose_frontend:main',
            'unknown_pose_validation_observer = '
            'my_epuck_project.unknown_pose_validation_observer:main',
            'frontier_proposal_adapter = '
            'my_epuck_project.frontier_proposal_adapter:main',
            'distributed_frontier_assignment = '
            'my_epuck_project.distributed_frontier_assignment:main',
            'minimal_frontier_allocator = '
            'my_epuck_project.minimal_frontier_allocator:main',
            'cooperative_experiment_logger = '
            'my_epuck_project.cooperative_experiment_logger:main',
            'robot2_resilient_mode_manager = '
            'my_epuck_project.robot2_resilient_mode_manager:main',
        ],
    },
)
