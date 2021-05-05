cmake_minimum_required(VERSION 3.10)
project(rstar_mrastar_comparison)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -Os")

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMake/")
find_package(Catch2 REQUIRED)

enable_testing()

add_subdirectory(common)
