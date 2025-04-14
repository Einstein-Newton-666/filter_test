from setuptools import find_packages
from setuptools import setup

setup(
    name='filter_test',
    version='0.0.0',
    packages=find_packages(
        include=('filter_test', 'filter_test.*')),
)
