#include "open.h"
#include "position.h"

#include <catch2/catch.hpp>
#include <memory>


TEST_CASE("check OpenSet") {
    using namespace heuristicsearch;
    auto open = OpenSet<Position, double>();
    
    auto n1 = Position{1, 1};
    REQUIRE(open.isEmpty());
    REQUIRE(open.size() == 0);
    open.addNodeOrDecreasePriority(n1, 15);
    REQUIRE(!open.isEmpty());
    REQUIRE(open.size() == 1);
    auto [minn, minf] = open.popMin();
    REQUIRE(open.isEmpty());
    REQUIRE(minf == 15);
    REQUIRE(minn == n1);
    
    open.addNodeOrDecreasePriority(n1, 15);
    open.addNodeOrDecreasePriority(n1, 5);
    REQUIRE(open.size() == 1);
    std::tie(minn, minf) = open.popMin();
    REQUIRE(minf == 5);
    REQUIRE(minn == n1);
    REQUIRE(open.isEmpty());

    auto n2 = Position{2, 2};
    open.addNodeOrDecreasePriority(n1, 5);
    open.addNodeOrDecreasePriority(n2, -2);
    REQUIRE(open.size() == 2);
    std::tie(minn, minf) = open.popMin();
    REQUIRE(minn == n2);
    std::tie(minn, minf) = open.popMin();
    REQUIRE(minn == n1);
    REQUIRE(open.isEmpty());
}
