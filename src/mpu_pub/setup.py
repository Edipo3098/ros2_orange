from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mpu_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add the launch and config directories to be installed
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),  # <- esto incluye controllers.yaml
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edipo',
    maintainer_email='felipe.porras.014@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpu_publisher = mpu_pub.mpu_publisher:main',
            'cog_calc = mpu_pub.cog_calc:main',
            'cog_calc2 = mpu_pub.cog_calc2:main',
            'efk_estimator_double = mpu_pub.efk_estimator_double:main',
            'efk_estimator = mpu_pub.efk_estimator:main',
            'ukf_estimator = mpu_pub.ukf_estimator:main',
            'ukf_estimator2 = mpu_pub.ukf_estimator2:main',
            'particle_filter = mpu_pub.particle_filter:main',
            'efk_estimator_2 = mpu_pub.efkEstimator_complementaryFilter:main',
            'ukf_estimator_pf = mpu_pub.ukf_estimator_pf:main',
            'motor_subscriber = mpu_pub.motor_subscriber:main',
            'motor_subscriber2 = mpu_pub.motor_subscriber2:main',
            'data_calibrated_mpu = mpu_pub.data_calibrated_mpu:main',
            'data_calibrated_just_offset= mpu_pub.data_calibrated_just_offset:main',
            'imu_bridge = mpu_pub.imu_bridge:main',
        ],
    },
)
