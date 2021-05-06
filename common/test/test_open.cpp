#include "open.h"
#include "node.h"

#include <catch2/catch.hpp>

TEST_CASE("check OpenSet") {
    using namespace heuristicsearch;
    auto open = OpenSet<SearchNode, double>();
    auto f = [](auto sn) -> double { return sn.gvalue + sn.hvalue; };
    
    auto n1 = SearchNode{1, 1, 10, 5};
    REQUIRE(open.isEmpty());
    REQUIRE(open.size() == 0);
    open.addNodeOrDecreasePriority(n1, f(n1));
    REQUIRE(!open.isEmpty());
    REQUIRE(open.size() == 1);
    auto [minn, minf] = open.popMin();
    REQUIRE(open.isEmpty());
    REQUIRE(minf == f(n1));
    REQUIRE(minn == n1);
    
    open.addNodeOrDecreasePriority(n1, f(n1));
    n1.hvalue -= 10;
    open.addNodeOrDecreasePriority(n1, f(n1));
    REQUIRE(open.size() == 1);
    std::tie(minn, minf) = open.popMin();
    REQUIRE(minf == f(n1));
    REQUIRE(minn == n1);
    REQUIRE(open.isEmpty());

    auto n2 = SearchNode{2, 2, 0, -123, static_cast<Position>(n1)};
    open.addNodeOrDecreasePriority(n1, f(n1));
    open.addNodeOrDecreasePriority(n2, f(n2));
    REQUIRE(open.size() == 2);
    std::tie(minn, minf) = open.popMin();
    REQUIRE(minn == n2);
    std::tie(minn, minf) = open.popMin();
    REQUIRE(minn == n1);
    REQUIRE(open.isEmpty());
}
