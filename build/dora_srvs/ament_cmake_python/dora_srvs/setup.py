from setuptools import find_packages
from setuptools import setup

setup(
    name='dora_srvs',
    version='0.0.0',
    packages=find_packages(
        include=('dora_srvs', 'dora_srvs.*')),
)
