#ifndef MRASTAR_MRASTAR_H
#define MRASTAR_MRASTAR_H

#include "heuristic_algo.h"
#include "map.h"
#include "position.h"

#include <vector>
#include <set>
#include <optional>


class MRAStar {
public:
    explicit MRAStar(const std::set<int>& cellSizes, double weight = 2.0, double suboptimalityCoef = 3.0);

    std::optional<heuristicsearch::HeuristicAlgoResult> operator()(
            const heuristicsearch::Map& map,
            heuristicsearch::Position level, heuristicsearch::Position goalPos,
            heuristicsearch::Metric heuristic,
            heuristicsearch::Metric dist = heuristicsearch::EuclideanDistance
    ) const;

protected:
    std::vector<int> cellSizes_;
    double weight_;
    double suboptimalityCoef_;
};

#endif
