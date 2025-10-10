from setuptools import find_packages, setup

package_name = 'mapReader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'
                      'rclpy',
                    'cv_bridge',
                    'numpy',
                    'ultralytics',
                    'opencv-python'],
    zip_safe=True,
    maintainer='robotics-pc-20',
    maintainer_email='viswachintu425@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_reader = mapReader.map_reader:main',
            'cmd_vel_to_rotor = mapReader.cmd_vel_to_rotor:main',
            'joy_drone_control = mapReader.joy_drone_controller:main',
            'map_yaml = mapReader.map_to_yaml:main',
            'dummy_writer = mapReader.dummy_writer:main',
        ],
    },
)
