#include "multi_resolution_map.h"

using namespace heuristicsearch;


MultiResolutionMap::MultiResolutionMap(const heuristicsearch::Map &map) : Map(map) {}

bool MultiResolutionMap::isTraversable(heuristicsearch::Position pos, int cellSize) const {
    return pos.row % cellSize == 0 && pos.col % cellSize == 0 && Map::isTraversable(pos);
}

std::vector<heuristicsearch::Position>
MultiResolutionMap::getNeighbors(heuristicsearch::Position pos, int cellSize) const {
    std::vector<heuristicsearch::Position> result;
    result.reserve(8);
    for (int dr = -1; dr <= 1; dr++) {
        for (int dc = -1; dc <= 1; dc++) {
            if (dr == 0 && dc == 0) continue;
            heuristicsearch::Position np{pos.row + dr * cellSize, pos.col + dc * cellSize};
            if (!inBounds(np) || !Map::isTraversable(np)) continue;
            bool traversable = true;
            for (int step = 1; step <= cellSize; step++) {
                heuristicsearch::Position pp1{pos.row + dr * step, pos.col + dc * step},
                                          pp2{pos.row + dr * (step - 1), pos.col + dc * step},
                                          pp3{pos.row + dr * step, pos.col + dc * (step - 1)};
                if (!(Map::isTraversable(pp1) && Map::isTraversable(pp2) && Map::isTraversable(pp3))) {
                    traversable = false;
                    break;
                }
            }
            if (traversable) {
                result.push_back(np);
            }
        }
    }
    return result;
}
