from setuptools import find_packages, setup

package_name = 'depth_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'huggingface_hub',
        'gradio',
        'gradio_imageslider',
        'matplotlib',
        'opencv-python',
        'torch',
        'torchvision',
        # 'huggingface_hub==0.12.1',  # pin to a version compatible with gradio
    ],
    zip_safe=True,
    maintainer='robot2',
    maintainer_email='viswachintu425@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rgb_to_depth = depth_package.rgb_to_depth:main',
        ],
    },
)
