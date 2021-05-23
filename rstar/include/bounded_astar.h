#ifndef RSTAR_BOUNDED_ASTAR_H
#define RSTAR_BOUNDED_ASTAR_H

#include "map.h"
#include "heuristic_algo.h"
#include "position.h"

#include <optional>
#include <cstddef>


struct BoundedAStarResult {
    std::optional<heuristicsearch::HeuristicAlgoResult> result;
    std::size_t doneExpansions;
    double goalCostLowerBound;
};

BoundedAStarResult BoundedAStar(
    double fvalThreshold, double weight,
    const heuristicsearch::Map& map, 
    heuristicsearch::Position startPos, heuristicsearch::Position goalPos, 
    heuristicsearch::Metric heuristic,
    heuristicsearch::Metric dist);

#endif
