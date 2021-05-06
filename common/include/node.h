#ifndef COMMON_NODE_H
#define COMMON_NODE_H

#include "map.h"

#include <functional>


namespace heuristicsearch {

struct SearchNode : public Position {
    double gvalue = INFTY;
    double hvalue = INFTY;
    Position parent = NO_PARENT;

    static const Position NO_PARENT;
    static const double INFTY;
};

bool operator==(const SearchNode &a, const SearchNode &b);
bool operator<(const SearchNode &a, const SearchNode &b);

} // namespace heuristicsearch

namespace std {
using namespace heuristicsearch;

template<> 
struct hash<SearchNode> {
    std::size_t operator()(SearchNode const& s) const noexcept;
};    

} // namespace std

#endif
