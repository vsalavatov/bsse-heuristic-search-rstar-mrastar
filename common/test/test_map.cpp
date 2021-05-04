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
            REQUIRE(map.isTraversable(i, j) == expected[i][j]);
}
