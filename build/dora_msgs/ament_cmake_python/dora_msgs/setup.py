from setuptools import find_packages
from setuptools import setup

setup(
    name='dora_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('dora_msgs', 'dora_msgs.*')),
)
