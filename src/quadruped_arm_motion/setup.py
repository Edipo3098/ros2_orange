from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quadruped_arm_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required resource file for ROS2 package indexing
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # Install the package.xml file
        ('share/' + package_name, ['package.xml']),
        
        # Install all launch files (py or other formats)
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        
        # Install URDF files (and any other related files)
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        
        # Install RViz config files (optional)
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edipo',
    maintainer_email='felipe.porras.014@gmail.com',
    description='Quadruped Arm Motion Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control = quadruped_arm_motion.motor_control:main',
            'motor_subscriber = quadruped_arm_motion.motor_subscriber:main',
            'trayectory_planning = quadruped_arm_motion.trayectory_planning:main',
        ],
    },
)
