#include "closed.h"
#include "position.h"

#include <catch2/catch.hpp>
#include <memory>


TEST_CASE("check ClosedSet") {
    using namespace heuristicsearch;
    auto closed = ClosedSet<Position>();
    
    auto n1 = Position{1, 1};
    REQUIRE(closed.isEmpty());
    REQUIRE(closed.size() == 0);
    closed.add(n1);
    REQUIRE(!closed.isEmpty());
    REQUIRE(closed.size() == 1);
    closed.add(n1);
    REQUIRE(closed.size() == 1);
    REQUIRE(closed.contains(n1));
    
    auto n2 = Position{2, 2};
    REQUIRE(!closed.contains(n2));
    closed.add(n2);
    REQUIRE(closed.size() == 2);
    REQUIRE(closed.contains(n2));
}
