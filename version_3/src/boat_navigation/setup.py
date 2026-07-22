from setuptools import find_packages, setup

package_name = 'boat_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='LOS path following with PID heading and speed control.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_pose2d = boat_navigation.gazebo_pose2d:main',
            'los_path_follower = boat_navigation.los_path_follower:main',
            'trajectory_plotter = boat_navigation.trajectory_plotter:main',
        ],
    },
)
