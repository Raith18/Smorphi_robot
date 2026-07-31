#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""setup.py for the dataset_loader Python module (catkin_python_setup)."""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["dataset_loader"],
    package_dir={"": "src"},
)

setup(**d)
