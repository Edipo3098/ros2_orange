from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'quadruped_arm_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/.control_launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/.orange_pi_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edipo',
    maintainer_email='felipe.porras.014@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control = quadruped_arm_motion.motor_control:main',
            'motor_subscriber = quadruped_arm_motion.motor_subscriber:main',
            'trayectory_planning = quadruped_arm_motion.trayectory_planning:main'
        ],
    },
)
