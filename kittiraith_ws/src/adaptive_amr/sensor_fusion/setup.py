#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""setup.py for sensor_fusion (catkin_python_setup)."""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["sensor_fusion"],
    package_dir={"": "src"},
)

setup(**d)
