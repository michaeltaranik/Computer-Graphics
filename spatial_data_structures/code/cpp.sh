#! /bin/bash

echo "Compiling..."
# g++ main.cpp --std=c++11 -O3 -march=native -ffast-math
g++ main.cpp --std=c++11 -O3
echo "Compiling... Completed"

echo "Rendering..."
./a.out
echo "Rendering... Completed"
