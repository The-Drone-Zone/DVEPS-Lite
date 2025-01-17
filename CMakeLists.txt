cmake_minimum_required(VERSION 3.10)

# Define the project name
project(DVEPS-Lite)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)

# Set the build directory to 'build'
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

# Add the source directory
add_executable(drone source/main.cpp)

# Enable testing
enable_testing()

# Add a simple test for command-line arguments
add_test(
    NAME HelloWorldTest
    COMMAND drone 3 4
)
