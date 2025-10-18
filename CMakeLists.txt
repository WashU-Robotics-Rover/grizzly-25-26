cmake_minimum_required(VERSION 3.16)

# Project name and version
project(Grizzly VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(include)

# Add source files
add_library(grizzly_core
        src/grizzly/core/bus/scheduler.cpp
        include/grizzly/core/bus/scheduler.hpp
)

# Specify include directories for the library
target_include_directories(grizzly_core PUBLIC include)