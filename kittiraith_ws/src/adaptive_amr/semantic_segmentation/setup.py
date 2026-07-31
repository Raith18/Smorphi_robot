#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""setup.py for semantic_segmentation (catkin_python_setup)."""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["semantic_segmentation"],
    package_dir={"": "src"},
)

setup(**d)
