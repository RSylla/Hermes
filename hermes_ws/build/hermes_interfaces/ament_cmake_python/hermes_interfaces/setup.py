from setuptools import find_packages
from setuptools import setup

setup(
    name='hermes_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('hermes_interfaces', 'hermes_interfaces.*')),
)
