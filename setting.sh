#!/bin/bash

# Define your install destination
YOUR_INSTALL_DESTINATION="/home/kyoichi-sugahara/PhD_workspace/autogenu-jupyter/install"

# Create a build directory and navigate into it
mkdir build
cd build

# Run cmake with the specified install prefix
cmake .. -DCMAKE_INSTALL_PREFIX=$YOUR_INSTALL_DESTINATION

# Build and install the project
make install

# Navigate back to the project root directory
cd ..

# Install setuptools and the current directory
python3 -m pip install setuptools
python3 -m pip install .
