from setuptools import find_packages
from setuptools import setup

setup(
    name='mission_planner',
    version='0.0.1',
    packages=find_packages(
        include=('mission_planner', 'mission_planner.*')),
)
