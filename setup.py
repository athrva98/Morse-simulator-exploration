"""setup.py
setup.py for morse_simulator
"""
from __future__ import print_function, absolute_import, division

from setuptools import setup, find_packages, Extension

with open("README.md", "r") as f:
    long_description = f.read()

setup(
    name='morse_simulator',
    version='0.0.1',
    description='Frontier Based Exploration with the Morse Simulator for Massively parallel Quadrotor simulations on a Single Node',
    long_description=long_description,
    author='Athrva Pandhare',
    author_email='athrva98@gmail.com',
    url='https://github.com/NonStopEagle137/Morse-simulator-exploration',
    download_url='',
    license='MIT',
    install_requires=['numpy>=1.11.0',
                      'matplotlib==2.2.3',
                      'opencv-python',
                      'pyyaml==5.1'],
    package_data={'': ['input']},
    include_package_data=True,
        extras_require={
        'tests': ['pytest==4.3.0',
                  'pytest-pep8==1.0.6',
                  'pytest-xdist==1.26.1',
                  'pylint==1.9.2',
                  'astroid==1.6.5'
                  ],
    },
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
        'Topic :: Software Development :: Libraries',
        'Topic :: Software Development :: Libraries :: Python Modules'
    ],
    packages=find_packages(),
    ext_package='morse_simulator',
    ext_modules=[Extension('_exploration_cpp',
                           extra_compile_args=['-std=c++1y', '-O3', '-Wall', '-fpic'],
                           include_dirs=['deps/pybind11/include',
                                         'morse_simulator/cpp/inc'],
                           sources=[
                               'morse_simulator/cpp/src/exploration/astar.cpp',
                               'morse_simulator/cpp/src/exploration/collision.cpp',
                               'morse_simulator/cpp/src/exploration/util.cpp',
                               'morse_simulator/cpp/src/exploration/python.cpp'
                           ])]
)
