# README
Algorithms for frontier Based Exploration In the MORSE Simulator.

# Requirements

1. Ubuntu 18.04
2. root access

# Installation of Dependencies

Before you intall the package, you need to have the Morse-Simulator and compatible Blender installed.
I have included a bash file to install these dependencies.
1. ```chmod +x ./ubuntu_1804_installation.sh```
2. ```./ubuntu_1804_installation.sh```

Note that this script creates a virtual environment which has the dependencies for MORSE. To activate this venv,
1. ```pipenv shell```
Alternatively, directly run the files using
1. ```pipenv run abc.py```
this runs the file using pipenv environment.

# Test the installation of MORSE
After running the intallation script, run
1. ```pipenv run morse check```
You should not see any errors.

# Installation of the Package
Please do this after completing all the above steps.
This repository can be installed as a python package by,
```sudo python setup.py install```

# TODO
1. Add install script that installs MORSE simulator with dependencies. [DONE]
2. Complete the OpenAI Gym wrapper. [DONE]
# References

The exploration code is adapted from [https://github.com/braincorp/bc_exploration.git]
