#!/bin/bash

echo "Compiling..."
g++ main.cpp -O3 
echo "Compiling... Completed"
echo "Rendering..."
./a.out
echo "Rendering... Completed"

