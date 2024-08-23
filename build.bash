#!/usr/bin/bash

# create build folder
if [ ! -d "build" ]; then
  mkdir build
fi

# Setup the CMake environment
cmake -S . -B build -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_CXX_FLAGS="-Wno-error -Wno-deprecated-declarations" -DCMAKE_C_FLAGS="-Wno-error -Wno-deprecated-declarations"

# Build all
cmake --build build

# Create the folder for custom plugins if it does not exist
if [ ! -d "build/bin/mujoco_plugin" ]; then
  mkdir build/bin/mujoco_plugin
fi

# Copy the plugin to the plugin folder
cp build/lib/libsoil.so build/bin/mujoco_plugin
cp build/lib/libspid.so build/bin/mujoco_plugin
cp build/lib/libangle_monitor.so build/bin/mujoco_plugin