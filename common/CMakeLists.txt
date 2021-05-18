add_library(common STATIC 
    src/map.cpp
    src/scenario.cpp
    src/position.cpp
    src/astar.cpp
)
target_include_directories(common PUBLIC include/)

add_executable(common_unit_test
    test/test.cpp
    test/test_map.cpp
    test/test_scenario.cpp
    test/test_open.cpp
    test/test_closed.cpp
    test/test_astar.cpp
)
target_link_libraries(common_unit_test common Catch2::Catch2)
target_include_directories(common_unit_test PUBLIC include/)
