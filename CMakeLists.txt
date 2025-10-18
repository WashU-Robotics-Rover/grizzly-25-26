cmake_minimum_required(VERSION 3.16)

# Project name and version
project(Grizzly VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define source files
set(SOURCES
        src/grizzly/core/runtime/scheduler.cpp
        include/grizzly/core/runtime/scheduler.hpp
        src/grizzly/core/runtime/scheduler.cpp
        src/grizzly/main.cpp
)

# Add library target
add_library(grizzly_core ${SOURCES}
        src/grizzly/main.cpp)

# Specify include directories for the library
target_include_directories(grizzly_core PUBLIC include)

# Link required libraries (if needed)
find_package(Threads REQUIRED)
target_link_libraries(grizzly_core PRIVATE Threads::Threads)

# Add executable target
add_executable(grizzly_app src/grizzly/main.cpp)
target_link_libraries(grizzly_app PRIVATE grizzly_core)
target_include_directories(grizzly_app PRIVATE include)
