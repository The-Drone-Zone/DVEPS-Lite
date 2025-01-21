cmake_minimum_required(VERSION 3.15)

# Define the project name
project(DVEPS-Lite)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)

# Set the build directory to 'build'
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

# Add the source directory
add_executable(drone src/main.cpp)

# Enable testing
enable_testing()

# Add a simple test for command-line arguments
add_test(
    NAME HelloWorldTest
    COMMAND drone 3 4
)

#### ADDED BY AIDAN (BELOW) ####

# Include directories
include_directories(include)

# Add the source file
add_library(math_functions src/math_functions.cpp)



# # Add Google Test
# include(FetchContent)
# FetchContent_Declare(
#   googletest
#   URL https://github.com/google/googletest/archive/refs/tags/release-1.13.0.zip
# )
# FetchContent_MakeAvailable(googletest)

# # Add the test executable
# add_executable(test_math_functions tests/test_math_functions.cpp)
# target_link_libraries(test_math_functions math_functions gtest gtest_main)

# Add sources
set(SOURCES
    src/math_functions.cpp
)

# Add test executable
add_executable(test_math_functions
    tests/test_math_functions.cpp
    ${SOURCES}
)

# Find and link Google Test
find_package(GTest REQUIRED)
target_link_libraries(test_math_functions PRIVATE GTest::gtest GTest::gtest_main)

# Add the test
add_test(NAME TestMathFunctions COMMAND test_math_functions)