#include "map.h"

#include <catch2/catch.hpp>

TEST_CASE("check MovingAI map loader") {
    using namespace heuristicsearch;
    Map map = Map::fromMovingAI("dataset/sample-moving-ai.map");
    REQUIRE(map.getHeight() == 4);
    REQUIRE(map.getWidth() == 5);
    std::vector<std::vector<bool>> expected = {
        {0, 0, 0, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0}
    };
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 5; ++j)
            REQUIRE(map.isTraversable(Position{i, j}) == expected[i][j]);
}

TEST_CASE("check Map path validation") {
    using namespace heuristicsearch;
    Map map = Map::fromMovingAI("dataset/sample-moving-ai.map");
    REQUIRE(
        validatePath(map, std::vector<Position>{})
    );
    REQUIRE(
        validatePath(map, std::vector<Position>{
            {1, 1}
        })
    );
    REQUIRE(
        validatePath(map, std::vector<Position>{
            {1, 1},
            {2, 1},
            {2, 2},
            {2, 3},
            {1, 3}
        })
    );


    REQUIRE(
        !validatePath(map, std::vector<Position>{
            {1, 1},
            {1, 3}
        })
    );
    REQUIRE(
        !validatePath(map, std::vector<Position>{
            {1, 1},
            {2, 2},
            {2, 3},
            {1, 3}
        })
    );
}

TEST_CASE("check path length calc") {
    using namespace heuristicsearch;
    REQUIRE(
        fabs(pathLength(std::vector<Position>{
            {1, 1},
            {2, 1},
            {2, 2},
            {2, 3},
            {1, 3}
        }) - 4.0) < 1e-9
    );
}
