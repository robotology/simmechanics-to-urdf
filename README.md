simmechanics-to-urdf
====================
[![Build Status](https://travis-ci.org/robotology-playground/simmechanics-to-urdf.svg?branch=master)](https://travis-ci.org/robotology-playground/simmechanics-to-urdf)

Script for converting first-generation ( PhysicalModelingXMLFile ) SimMechanics XML files to URDF ( http://wiki.ros.org/urdf ) .

Based on the [original version](http://wiki.ros.org/simmechanics_to_urdf) by [David V. Lu!!](http://www.cse.wustl.edu/~dvl1/).

#### Dependencies
- [lxml](http://lxml.de/)
- [PyYAML](http://pyyaml.org/)
- [NumPy](http://www.numpy.org/)
- [urdf_parser_py](https://github.com/ros/urdfdom/tree/master/urdf_parser_py)

## Installation

#### Debian/Ubuntu
##### Install dependencies
Install the necessary dependencies with apt-get:
~~~
sudo apt-get install python-lxml python-yaml python-numpy
~~~
You can install `urdf_parser_py` from the urdfdom git repository:
~~~
git clone https://github.com/ros/urdfdom
cd urdfdom/urdf_parser_py
sudo python setup.py install
~~~
##### Use the script
You can call the script:
~~~
python convert.py {SimMechanics XML filename} [configfile] {xml|graph|none}
~~~
The third argument to the script is the output. Selecting graph the script will output a graphviz representation 
of the SimMechanics model, useful for debugging, while selecting xml it will output the converted URDF.
