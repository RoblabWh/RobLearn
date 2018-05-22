#!/bin/bash

mkdir pysim2d
cd pysim2d
cmake ../../Simulation2d -DCMAKE_BUILD_TYPE=Release && make -j