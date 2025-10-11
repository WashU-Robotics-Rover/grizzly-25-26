cmake_minimum_required(VERSION 3.16)

# Project name and version
project(Grizzly)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add include directories
include_directories(core/include
                    control/include)

# Add source files
file(GLOB_RECURSE SOURCES "core/src/*.cpp"
                          "control/src/host/*.cpp")

# Link libraries (if needed)
# target_link_libraries(component_manager <library_name>)

# Add an executable (if you have a main.cpp)
add_executable(Grizzly core/src/main.cpp ${SOURCES}
         core/src/main.cpp)
# target_link_libraries(my_executable component_manager)