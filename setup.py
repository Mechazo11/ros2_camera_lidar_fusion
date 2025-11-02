from setuptools import find_packages, setup
from glob import glob

package_name = 'ros2_camera_lidar_fusion'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mechazo11',
    maintainer_email='azmyin12@gmail.com',
    description='ROS2 Package to calculate the intrinsic and extrinsic camera calibration. This fork adds a few fine-grained control features along with helpful documentations ',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_intrinsic_camera_calibration = ros2_camera_lidar_fusion.get_intrinsic_camera_calibration:main',
            'get_extrinsic_camera_calibration = ros2_camera_lidar_fusion.get_extrinsic_camera_calibration:main',
            'save_data = ros2_camera_lidar_fusion.save_sensor_data:main',
            'extract_points = ros2_camera_lidar_fusion.extract_points:main',
            'lidar_camera_projection = ros2_camera_lidar_fusion.lidar_camera_projection:main',
        ],
    },
)
