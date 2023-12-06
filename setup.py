from setuptools import setup

package_name = 'vive_tracker_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='flypulator',
    maintainer_email='flypulator@tu-dresden.de',
    description='adaption of https://github.com/moon-wreckers/vive_tracker to ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = vive-tracker-ros2.vive_tracker:main',
            'talker = vive-tracker-ros2.vive_world:main',
        ],
    },
)
