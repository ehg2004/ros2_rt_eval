from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2_rt_eval_dep',
    version='0.0.0',
    packages=find_packages(
        include=('ros2_rt_eval_dep', 'ros2_rt_eval_dep.*')),
)
