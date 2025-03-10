cmake_minimum_required(VERSION 3.10)
project(MyProject)

set(CMAKE_CXX_STANDARD 17)

# Add debug options
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

# Include directories if needed
include_directories(${PROJECT_SOURCE_DIR}/include)

# Add source files
file(GLOB SOURCES "${PROJECT_SOURCE_DIR}/*.cpp")

add_executable(MyProject ${SOURCES})