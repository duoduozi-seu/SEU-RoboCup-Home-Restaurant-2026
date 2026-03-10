#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""为 catkin 暴露纯 Python 核心模块。"""

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=["seu_look_for_object_core"],
    package_dir={"": "src"},
)

setup(**setup_args)
