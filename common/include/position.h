#ifndef COMMON_POSITION_H
#define COMMON_POSITION_H

#include <functional>
#include <memory>
#include <ostream>

namespace heuristicsearch {

struct Position {
    int row;
    int col;
};
bool operator==(const Position &a, const Position &b);
bool operator<(const Position &a, const Position &b);
std::ostream& operator<<(std::ostream& o, const Position &p);

} // namespace heuristicsearch

namespace std {
using namespace heuristicsearch;

template<> 
struct hash<Position> {
    std::size_t operator()(Position const& s) const;
};

} // namespace std

#endif
