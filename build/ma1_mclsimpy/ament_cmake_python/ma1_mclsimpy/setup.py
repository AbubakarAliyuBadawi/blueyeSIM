from setuptools import find_packages
from setuptools import setup

setup(
    name='ma1_mclsimpy',
    version='0.0.0',
    packages=find_packages(
        include=('ma1_mclsimpy', 'ma1_mclsimpy.*')),
)
