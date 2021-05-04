add_library(common STATIC src/map.cpp)
target_include_directories(common PUBLIC include/)

add_executable(unit_test
    test/test.cpp
    test/test_map.cpp
)
target_link_libraries(unit_test common Catch2::Catch2)
target_include_directories(unit_test PUBLIC include/)
