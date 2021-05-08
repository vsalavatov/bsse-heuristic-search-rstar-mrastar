#include "scenario.h"

#include <catch2/catch.hpp>
#include <cmath>

TEST_CASE("check MovingAI scenario loader") {
    using namespace heuristicsearch;
    auto scenarios = Scenario::fromMovingAI("dataset/sample-moving-ai.map.scen");
    auto map = Map::fromMovingAI("dataset/sample-moving-ai.map");

    REQUIRE(scenarios.size() == 2);

    REQUIRE(scenarios[0].start == Position{1, 1});
    REQUIRE(scenarios[0].finish == Position{1, 3});
    REQUIRE(fabs(scenarios[0].getBestPossibleDistance(map) - 4.0) < 1e-9);

    REQUIRE(scenarios[1].start == Position{2, 1});
    REQUIRE(scenarios[1].finish == Position{2, 3});
    REQUIRE(fabs(scenarios[1].getBestPossibleDistance(map) - 2.0) < 1e-9);
}
