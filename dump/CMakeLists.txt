include(${PROJECT_SOURCE_DIR}/CMakeInclude/CPM.cmake)

add_executable(dump
    src/main.cpp
    src/open_dump.cpp
    src/rss_check.cpp
)
target_link_libraries(dump common rstar mrastar cxxopts::cxxopts)
target_include_directories(dump PUBLIC
    include/
    ../common/include/
    ../rstar/include/
    ../mrastar/include/
)

CPMAddPackage("gh:jarro2783/cxxopts#dd45a0801c99d62109aaa29f8c410ba8def2fbf2")
