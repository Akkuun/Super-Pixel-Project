#!/bin/bash

# Set the project directory
PROJECT_DIR=$(dirname "$0")

# Set the build directory
BUILD_DIR="$PROJECT_DIR/build"

# Create the build directory if it doesn't exist
mkdir -p "$BUILD_DIR"

# Navigate to the build directory
cd "$BUILD_DIR"

# Run CMake to configure the project
cmake ..

# Build the project
make

# Run the executable
./turboPixelProjet