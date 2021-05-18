add_library(rstar STATIC
    src/rstar.cpp
    src/bounded_astar.cpp
)
target_link_libraries(rstar common)
target_include_directories(rstar PUBLIC 
    include/ 
    ../common/include/
)

add_executable(rstar_unit_test
    test/test.cpp
    test/test_rstar.cpp
)
target_link_libraries(rstar_unit_test rstar Catch2::Catch2)
target_include_directories(rstar_unit_test PUBLIC 
    include/ 
    ../common/include/
)
