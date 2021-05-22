#ifndef MRASTAR_MULTI_RESOLUTION_MAP_H
#define MRASTAR_MULTI_RESOLUTION_MAP_H

#include "map.h"
#include "position.h"


class MultiResolutionMap : public heuristicsearch::Map {
public:
    MultiResolutionMap(const MultiResolutionMap&) = default;
    MultiResolutionMap(MultiResolutionMap&&) = default;
    ~MultiResolutionMap() = default;

    explicit MultiResolutionMap(const heuristicsearch::Map& map);

    bool isTraversable(heuristicsearch::Position pos, int cellSize) const;

    std::vector<heuristicsearch::Position> getNeighbors(heuristicsearch::Position pos, int cellSize) const;
};

#endif
