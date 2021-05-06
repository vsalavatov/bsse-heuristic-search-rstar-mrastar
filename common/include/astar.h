#ifndef COMMON_ASTAR_H
#define COMMON_ASTAR_H

#include "map.h"
#include "position.h"
#include "scenario.h"
#include "heuristic_algo.h"
#include "open.h"
#include "closed.h"

#include <tuple>
#include <vector>
#include <memory>
#include <functional>
#include <unordered_map>
#include <optional>


namespace heuristicsearch {

std::optional<HeuristicAlgoResult> AStar(
    const Map& map, 
    Position startPos, Position goalPos, 
    Metric heuristic,
    Metric dist = EuclideanDistance
);

}

#endif
