cmake_minimum_required(VERSION 3.14)
project(CurveReconstruction)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find CGAL
set(CMAKE_PREFIX_PATH "/opt/homebrew/opt/cgal" ${CMAKE_PREFIX_PATH})
find_package(CGAL REQUIRED)

# Include directories
include_directories(src)

# Source files
set(SOURCES
    src/main.cpp
    src/Crust.cpp
    src/Peeling.cpp
    src/Utils.cpp
)

# Executable
add_executable(CurveReconstruction ${SOURCES})

# Link libraries
target_link_libraries(CurveReconstruction PRIVATE CGAL::CGAL)
