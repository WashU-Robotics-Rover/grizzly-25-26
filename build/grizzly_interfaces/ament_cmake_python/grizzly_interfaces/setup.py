from setuptools import find_packages
from setuptools import setup

setup(
    name='grizzly_interfaces',
    version='0.1.0',
    packages=find_packages(
        include=('grizzly_interfaces', 'grizzly_interfaces.*')),
)
