#!/bin/bash

rm -rf build/

mkdir build

cd build

cmake ..

cmake --build .

./lateral_control