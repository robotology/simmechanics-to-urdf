simmechanics-to-urdf
====================
[![Build Status](https://travis-ci.org/robotology-playground/simmechanics-to-urdf.svg?branch=master)](https://travis-ci.org/robotology-playground/simmechanics-to-urdf)


This tool was developed to convert CAD models to URDF ( http://wiki.ros.org/urdf )  models semi-automatically. It makes use of the XML files exported by the SimMechanics Link . Mathworks, makers of SimMechanics, have developed plugins for a couple of leading CAD programs, including SolidWorks, ProEngineer and Inventor. 

More specifically, this library at the moment just support first-generation SimMechanics XML files, that in MathWorks documentation are also referred as `PhysicalModelingXMLFile` .

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
cd 
sudo python setup.py install
~~~

##### Install simmechanics-to-urdf
You can then install `simmechanics-to-urdf`:
~~~
git clone https://github.com/robotology-playground/simmechanics-to-urdf
cd simmechanics-to-urdf
sudo python setup.py install
~~~

## How it works
The SimMechanics Link creates an XML file (PhysicalModelingXMLFile) and a collection of STL files. The XML describes all of the bodies, reference frames, inertial frames and joints for the model. The simmechanics_to_urdf script takes this information and converts it to a URDF. However, there are some tricks and caveats, which can maneuvered using a couple of parameters files. Not properly specifing this parameters files will result in a model that looks correct when not moving, but possibly does not move correctly. 

### Tree vs. Graph

URDF allows robot descriptions to only follow a tree structure, meaning that each link/body can have only one parent, and its position is dependent on the position of its parent and the position of the joint connecting it to its parent. This forces URDF descriptions to be in a tree structure.

CAD files do not generally follow this restriction; one body can be dependent on multiple bodies, or at the very least, aligned to multiple bodies.

This creates two problems.

  - The graph must be converted into a tree structure. This is done by means of a breadth first traversal of the graph, starting from the root link. However, this sometimes leads to improper dependencies, which can be corrected with the parameter file, as described below.

  - Fixed joints in CAD are not always fixed in the exported XML. To best understand this, consider the example where you have bodies A, B and C, all connected to each other. If C is connected to A in such a way that it is constrained in the X and Z dimensions, and C is connected to B so that it is constrained in the Y dimension, then effectively, C is fixed/welded to both of those bodies. Thus removing the joint between B and C (which is needed to make the tree structure) frees up the joint. This also can be fixed with the parameter file. 

## Use the script
You can call the script:
~~~
simmechanics_to_urdf {SimMechanics XML filename} --yaml [yaml_configfile] --csv-joint csv_joints_configfile --output {xml|graph|none}
~~~
The `--output` options defines the output. Selecting graph the script will output a graphviz .dot representation 
of the SimMechanics model, useful for debugging, while selecting xml it will output the converted URDF.

### Configuration files

#### YAML Parameter File

#### CSV  Parameter File
