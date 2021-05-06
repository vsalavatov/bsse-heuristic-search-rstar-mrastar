#ifndef COMMON_HEURISTIC_ALGO_H
#define COMMON_HEURISTIC_ALGO_H

#include "position.h"

#include <vector>


namespace heuristicsearch {

struct HeuristicAlgoResult {
    std::vector<Position> path;
    double distance;
};

}

#endif
