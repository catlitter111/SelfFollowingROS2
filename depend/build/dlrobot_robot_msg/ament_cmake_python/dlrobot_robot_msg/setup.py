from setuptools import find_packages
from setuptools import setup

setup(
    name='dlrobot_robot_msg',
    version='0.0.0',
    packages=find_packages(
        include=('dlrobot_robot_msg', 'dlrobot_robot_msg.*')),
)
