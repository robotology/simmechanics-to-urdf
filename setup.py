#!/usr/bin/env python

import sys
from setuptools import setup, find_packages


setup(name='simmechanics_to_urdf',
      version='0.3.0',
      description='Converts SimMechanics XML to URDF',
      author='Silvio Traversaro, David V. Lu',
      author_email='pegua1@gmail.com',
      url='https://github.com/robotology-playground/simmechanics-to-urdf',
      packages=['simmechanics_to_urdf'],
      licence='BSD',
      classifiers=[
        "Development Status :: 3 - Alpha",
        "License :: OSI Approved :: BSD License",
        "Operating System :: OS Independent",
        'Programming Language :: Python :: 2.6',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3.3'
      ],
      install_requires=[
        "lxml",
        "numpy",
        "PyYAML >= 3.10"
      ],
      entry_points={
        'console_scripts': [
           'simmechanics_to_urdf = simmechanics_to_urdf.firstgen:main',
         ]
      }
     )
