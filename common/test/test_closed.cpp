#include "closed.h"
#include "node.h"

#include <catch2/catch.hpp>

TEST_CASE("check ClosedSet") {
    using namespace heuristicsearch;
    auto closed = ClosedSet<SearchNode>();
    
    auto n1 = SearchNode{1, 1, 10, 5};
    REQUIRE(closed.isEmpty());
    REQUIRE(closed.size() == 0);
    closed.add(n1);
    REQUIRE(!closed.isEmpty());
    REQUIRE(closed.size() == 1);
    closed.add(n1);
    REQUIRE(closed.size() == 1);
    REQUIRE(closed.contains(n1));
    
    auto n2 = SearchNode{2, 2, 0, -123, static_cast<Position>(n1)};
    REQUIRE(!closed.contains(n2));
    closed.add(n2);
    REQUIRE(closed.size() == 2);
    REQUIRE(closed.contains(n2));
}
