cmake_minimum_required(VERSION 3.10)

# Project name
project(TestFonction)

# Compiler and linker flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++17 -O3 -march=native -flto -funroll-loops -fopenmp")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto -fopenmp")

# Find FLTK
find_package(FLTK REQUIRED)

# Source files
set(SRC_FILES
        ${CMAKE_SOURCE_DIR}/FonctionsImages.cpp
        ${CMAKE_SOURCE_DIR}/main.cpp
        ${CMAKE_SOURCE_DIR}/Image.cpp
        ${CMAKE_SOURCE_DIR}/image_ppm.cpp
)

# Add executable
add_executable(turboPixelProjet ${SRC_FILES})

# Link FLTK
target_link_libraries(turboPixelProjet fltk)

# Profiling mode
add_custom_target(profile
        COMMAND ${CMAKE_COMMAND} -DCMAKE_CXX_FLAGS="-fprofile-generate" -P ${CMAKE_BINARY_DIR}/CMakeFiles/profile.cmake
        COMMAND ./turboPixelProjet
        COMMAND mv *.gcda *.gcno ${CMAKE_BINARY_DIR}/obj 2>/dev/null || true
)

# Optimize mode
add_custom_target(optimize
        COMMAND ${CMAKE_COMMAND} -DCMAKE_CXX_FLAGS="-fprofile-use" -P ${CMAKE_BINARY_DIR}/CMakeFiles/optimize.cmake
)

# Clean up
add_custom_target(clean-all
        COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/obj
        COMMAND ${CMAKE_COMMAND} -E remove turboPixelProjet *.gcda *.gcno
)

# Include directories
include_directories(${CMAKE_SOURCE_DIR})

# Create obj directory
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/obj)