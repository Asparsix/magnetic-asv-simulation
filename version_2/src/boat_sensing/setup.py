from setuptools import find_packages, setup

package_name = 'boat_sensing'

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
    description='Magnetometer driver and filter chain for the ASV stack.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mag_driver = boat_sensing.mag_driver:main',
            'mag_filter = boat_sensing.mag_filter:main',
            'calibration_node = boat_sensing.calibration_node:main',
        ],
    },
)
