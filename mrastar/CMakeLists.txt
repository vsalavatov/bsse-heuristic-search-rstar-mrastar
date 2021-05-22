add_library(mrastar STATIC
    src/mrastar.cpp
    src/multi_resolution_map.cpp
)
target_link_libraries(mrastar common)
target_include_directories(mrastar PUBLIC
    include/
    ../common/include/
)

add_executable(mrastar_unit_test
    test/test.cpp
    test/test_mrastar.cpp
)
target_link_libraries(mrastar_unit_test mrastar Catch2::Catch2)
target_include_directories(mrastar_unit_test PUBLIC
    include/
    ../common/include/
)
