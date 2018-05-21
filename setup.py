#!/usr/bin/env python
# -*- coding:utf-8 -*-
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
from subprocess import *
import os

d = generate_distutils_setup(
    packages=['robocup2018_gpsr'],
    package_dir={'': 'src'},
)

setup(**d)

#ここから実行権限を与えるscript
src_path = os.getcwd() + '/src/*'

cmd1 = 'chmod 755 ' + src_path
Popen(cmd1, shell=True)
