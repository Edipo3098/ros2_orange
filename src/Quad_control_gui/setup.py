from setuptools import find_packages, setup

package_name = 'Quad_control_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'control_gui = Quad_control_gui.control_gui:main',  # ROS 2 executable exceutable = package_name + file_name + function_name
            'control_robot_node = Quad_control_gui.control_robot_node:main',  # ROS 2 executable exceutable = package_name + file_name + function_name
        ],
    },
)
