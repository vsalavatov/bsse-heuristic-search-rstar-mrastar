#include "astar.h"

#include <catch2/catch.hpp>

TEST_CASE("check A* with 0-heuristic") {
    using namespace heuristicsearch;
    auto map = Map::fromMovingAI("dataset/sample-moving-ai.map");
    auto optresult = AStar(map, {1, 1}, {1, 3}, [](auto s, auto g) { return 0; });
    REQUIRE(optresult.has_value());
    auto result = *optresult;
    REQUIRE(fabs(result.distance - 4.0) < 1e-9);
    REQUIRE(result.path.size() > 2);
    REQUIRE(result.path[0] == Position{1, 1});
    REQUIRE(result.path.back() == Position{1, 3});
    REQUIRE(validatePath(map, result.path));
}
