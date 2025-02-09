cmake_minimum_required(VERSION 3.15)

# Define the project name
project(DVEPS-Lite)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)

# Set the build directory to 'build'
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

# Include directories
include_directories(include)

# Define source files
set(SOURCES
    src/math_functions.cpp
)

# Add the source file as a library
add_library(math_functions ${SOURCES})

# Add main executable
add_executable(drone src/main.cpp)

# Add test executable
add_executable(test_math_functions
    tests/test_math_functions.cpp
    ${SOURCES}
)

# Find and link libraries
find_package(GTest REQUIRED)
target_link_libraries(test_math_functions PRIVATE math_functions GTest::gtest GTest::gtest_main)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(drone ${OpenCV_LIBS})



# Add the test
add_test(NAME TestMathFunctions COMMAND test_math_functions)
