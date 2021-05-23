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
        auto optres = RStar(5.0, 3, 1.0)(map, scen.start, scen.finish, hfunc);
        REQUIRE(optres.has_value());
        auto result = *optres;
        REQUIRE(result.path[0] == scen.start);
        REQUIRE(result.path.back() == scen.finish);
        REQUIRE(validatePath(map, result.path));
        REQUIRE(fabs(result.distance - scen.getBestPossibleDistance(map)) < 1e-7);
    }
}

TEST_CASE("check R* suboptimality") {
    using namespace heuristicsearch;
    auto map = Map::fromMovingAI("dataset/moving-ai-large-test.map");
    auto scenarios = Scenario::fromMovingAI("dataset/moving-ai-large-test.map.scen");
    for (std::size_t sc = 0; sc < scenarios.size(); sc++) {
        if (sc % 300 != 0) {
            continue;
        }
        auto& scen = scenarios[sc];
        auto hfunc = OctileDistance;
        for (double suboptimalityCoef : {3.0, 7.0}) {
            std::cerr << scen.start << ' ' << scen.finish;
            auto optres = RStar(20, 20, suboptimalityCoef, 5.0)(
                map, scen.start, scen.finish, hfunc 
            );
            REQUIRE(optres.has_value());
            auto result = *optres;
            REQUIRE(result.path[0] == scen.start);
            REQUIRE(result.path.back() == scen.finish);
            REQUIRE(validatePath(map, result.path));
            REQUIRE(result.distance <= scen.getBestPossibleDistance(map) * suboptimalityCoef);
            std::cerr << ' ' << scen.getBestPossibleDistance(map) << ' ' << result.distance << std::endl;
        }
    }
}
