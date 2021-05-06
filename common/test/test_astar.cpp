#include "astar.h"

#include <catch2/catch.hpp>
#include <iostream>

TEST_CASE("check A* with 0-heuristic") {
    using namespace heuristicsearch;
    auto map = Map::fromMovingAI("dataset/sample-moving-ai.map");
    auto optresult = AStar(map, {1, 1}, {1, 3}, [](auto, auto) { return 0; });
    REQUIRE(optresult.has_value());
    auto result = *optresult;
    REQUIRE(fabs(result.distance - 4.0) < 1e-9);
    REQUIRE(result.path.size() > 2);
    REQUIRE(result.path[0] == Position{1, 1});
    REQUIRE(result.path.back() == Position{1, 3});
    REQUIRE(validatePath(map, result.path));
}

TEST_CASE("check A* with euclidean heuristic") {
    using namespace heuristicsearch;
    auto map = Map::fromMovingAI("dataset/sample-moving-ai-2.map");
    auto scenarios = Scenario::fromMovingAI("dataset/sample-moving-ai-2.map.scen");
    for (auto &scen : scenarios) {
        auto hfunc = EuclideanDistance;
        auto optres = AStar(map, scen.start, scen.finish, hfunc);
        REQUIRE(optres.has_value());
        auto result = *optres;
        REQUIRE(result.path[0] == scen.start);
        REQUIRE(result.path.back() == scen.finish);
        REQUIRE(validatePath(map, result.path));
        REQUIRE(fabs(result.distance - scen.getBestPossibleDistance(map)) < 1e-7);
    }
}

TEST_CASE("check A* with octile heuristic") {
    using namespace heuristicsearch;
    auto map = Map::fromMovingAI("dataset/sample-moving-ai-2.map");
    auto scenarios = Scenario::fromMovingAI("dataset/sample-moving-ai-2.map.scen");
    for (auto &scen : scenarios) {
        auto hfunc = OctileDistance;
        auto optres = AStar(map, scen.start, scen.finish, hfunc);
        REQUIRE(optres.has_value());
        auto result = *optres;
        REQUIRE(result.path[0] == scen.start);
        REQUIRE(result.path.back() == scen.finish);
        REQUIRE(validatePath(map, result.path));
        REQUIRE(fabs(result.distance - scen.getBestPossibleDistance(map)) < 1e-7);
    }
}
