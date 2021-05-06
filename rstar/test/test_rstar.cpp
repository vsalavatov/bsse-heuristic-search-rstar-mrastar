#include "rstar.h"
#include "map.h"
#include "scenario.h"

#include <catch2/catch.hpp>
#include <iostream>

TEST_CASE("check R* with octile heuristic") {
    using namespace heuristicsearch;
    auto map = Map::fromMovingAI("dataset/sample-moving-ai-2.map");
    auto scenarios = Scenario::fromMovingAI("dataset/sample-moving-ai-2.map.scen");
    for (auto &scen : scenarios) {
        auto hfunc = OctileDistance;
        auto optres = RStar(5.0, 3, 10)(map, scen.start, scen.finish, hfunc);
        REQUIRE(optres.has_value());
        auto result = *optres;
        REQUIRE(result.path[0] == scen.start);
        REQUIRE(result.path.back() == scen.finish);
        REQUIRE(validatePath(map, result.path));
        REQUIRE(fabs(result.distance - scen.getBestPossibleDistance(map)) < 1e-7);
    }
}

