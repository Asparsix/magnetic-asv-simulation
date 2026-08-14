from setuptools import find_packages, setup

package_name = 'boat_mission'

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
    description='Mission manager and coverage/hunt path planning for magnetic ASV search.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager = boat_mission.mission_manager:main',
            'spiral_demo = boat_mission.spiral_demo:main',
            'verify_coordinator = boat_mission.verify_coordinator:main',
        ],
    },
)
