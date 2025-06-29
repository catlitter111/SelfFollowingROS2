from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'following_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='monster',
    maintainer_email='monster@todo.todo',
    description='Following robot with stereo vision capabilities',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_vision_node = following_robot.stereo_vision_node:main',
        ],
    },
)
