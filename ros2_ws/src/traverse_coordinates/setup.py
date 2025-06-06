from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'traverse_coordinates'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name,'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amit-singh',
    maintainer_email='amitsingh9994@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coordinates_publisher = traverse_coordinates.coordinates_publisher:main',
        ],
    },
)
