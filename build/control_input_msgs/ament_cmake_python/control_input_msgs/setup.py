from setuptools import find_packages
from setuptools import setup

setup(
    name='control_input_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('control_input_msgs', 'control_input_msgs.*')),
)
