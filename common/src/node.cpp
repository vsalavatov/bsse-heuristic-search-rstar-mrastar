#include "node.h"
#include "hashutils.h"

namespace heuristicsearch {

const double SearchNode::INFTY = 1e18;
const Position SearchNode::NO_PARENT = Position{-1234, -1234};

bool operator==(const SearchNode &a, const SearchNode &b) {
    return static_cast<Position>(a) == static_cast<Position>(b);
}
bool operator<(const SearchNode &a, const SearchNode &b) {
    return a.row < b.row || (a.row == b.row && a.col < b.col);
}

} // namespace heuristicsearch

namespace std {

std::size_t hash<SearchNode>::operator()(SearchNode const& s) const noexcept {
    std::size_t seed = 0;
    hash_combine(seed, s.row);
    hash_combine(seed, s.col);
    return seed;
}

} // namespace std
