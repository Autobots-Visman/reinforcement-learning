# The setup.py file is a standard way of defining how to install the package.

from setuptools import setup

setup(name="handy",
      version="0.1",
      install_requires=["gymnasium==0.29.1"]  # And any dependencies needed
)
