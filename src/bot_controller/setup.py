from setuptools import find_packages, setup
import os
package_name = 'bot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this block to include meshes
        ('share/' + package_name + '/meshes', [
            os.path.join('meshes', f)
            for f in os.listdir(os.path.join(os.path.dirname(__file__), 'meshes'))
            if f.endswith('.dae') or f.endswith('.png')
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot2',
    maintainer_email='viswachintu425@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_reader = bot_controller.depth_reader:main',
            'mark_pub = bot_controller.marker_publish:main',
            'cam_reader = bot_controller.cam_reader:main',
            'lidar_reader = bot_controller.lidar_reader:main',
        ],
    },
)
