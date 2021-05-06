#include "position.h"
#include "hashutils.h"

namespace heuristicsearch {

/**
 * Position
 */

bool operator==(const Position &a, const Position &b) {
    return a.row == b.row && a.col == b.col;
}

bool operator<(const Position &a, const Position &b) {
    return a.row < b.row || (a.row == b.row && a.col < b.col);
}

std::ostream& operator<<(std::ostream& o, const Position &p) {
    o << "[" << p.row << "," << p.col << "]";
    return o;
}

} // namespace heuristicsearch

namespace std {

std::size_t hash<Position>::operator()(Position const& s) const {
    std::size_t seed = 0;
    hash_combine(seed, s.row);
    hash_combine(seed, s.col);
    return seed;
}

} // namespace std
