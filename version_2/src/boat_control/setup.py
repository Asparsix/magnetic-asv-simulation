from setuptools import find_packages, setup

package_name = 'boat_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='Twist to differential-thrust mixer for the simulated boat.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'thrust_mixer = boat_control.mixer:main',
        ],
    },
)
