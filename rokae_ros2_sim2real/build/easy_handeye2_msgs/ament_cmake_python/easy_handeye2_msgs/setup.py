from setuptools import find_packages
from setuptools import setup

setup(
    name='easy_handeye2_msgs',
    version='0.5.0',
    packages=find_packages(
        include=('easy_handeye2_msgs', 'easy_handeye2_msgs.*')),
)
