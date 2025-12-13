from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- CRITICAL: You need BOTH of these lines ---
        # 1. Copies the launch files (System startup)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. Copies the dashboard files (OpenMCT)
        (os.path.join('share', package_name, 'dashboard'), glob('dashboard/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mehpara',
    maintainer_email='mehpara@todo.todo',
    description='Rover Control Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematics = rover_control.kinematics_node:main',
            'image_stats = rover_control.image_stats_node:main',
        ],
    },
)