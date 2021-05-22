#include "mrastar.h"
#include "map.h"
#include "scenario.h"

#include <catch2/catch.hpp>


TEST_CASE("check MRA* with octile heuristic") {
    using namespace heuristicsearch;
    auto map = Map::fromMovingAI("dataset/sample-moving-ai-2.map");
    auto scenarios = Scenario::fromMovingAI("dataset/sample-moving-ai-2.map.scen");
    for (auto &scen : scenarios) {
        auto hfunc = OctileDistance;
        auto optres = MRAStar({2, 3, 4}, 1.5, 1.5)(map, scen.start, scen.finish, hfunc);
        REQUIRE(optres.has_value());
        auto result = *optres;
        REQUIRE(result.path[0] == scen.start);
        REQUIRE(result.path.back() == scen.finish);
        REQUIRE(validatePath(map, result.path));
        REQUIRE(fabs(result.distance - scen.getBestPossibleDistance(map)) < 1e-7);
    }
}

TEST_CASE("check MRA* suboptimality") {
    using namespace heuristicsearch;
    auto map = Map::fromMovingAI("dataset/moving-ai-large-test.map");
    auto scenarios = Scenario::fromMovingAI("dataset/moving-ai-large-test.map.scen");
    for (std::size_t sc = 0; sc < scenarios.size(); sc++) {
        if (sc % 300 != 0) {
            continue;
        }
        auto& scen = scenarios[sc];
        auto hfunc = OctileDistance;
        for (double suboptimalityCoef : {1.1, 3.0, 7.0}) {
            auto optres = MRAStar({2, 4, 8, 16, 32, 64}, suboptimalityCoef, suboptimalityCoef)(
                map, scen.start, scen.finish, hfunc
            );
            REQUIRE(optres.has_value());
            auto result = *optres;
            REQUIRE(result.path[0] == scen.start);
            REQUIRE(result.path.back() == scen.finish);
            REQUIRE(validatePath(map, result.path));
            REQUIRE(result.distance <= scen.getBestPossibleDistance(map) * suboptimalityCoef);
        }
    }
}
