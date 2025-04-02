from setuptools import find_packages
from setuptools import setup

setup(
    name='package_with_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('package_with_interfaces', 'package_with_interfaces.*')),
)
