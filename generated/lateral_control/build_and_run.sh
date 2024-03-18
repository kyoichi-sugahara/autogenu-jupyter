#!/bin/bash

# Remove the existing build directory
rm -rf build/

# Create a new build directory
mkdir build

# Change into the new build directory
cd build

# Configure the project with CMake. This will look for a CMakeLists.txt file and generate a Makefile needed for building
cmake ..

# Build the project using the generated Makefile
cmake --build .

# If the build is successful, run the generated executable
./lateral_control