#ifndef RSTAR_RSTAR_H
#define RSTAR_RSTAR_H

#include <optional>
#include <cstddef>
#include <random>

#include "heuristic_algo.h"
#include "map.h"
#include "position.h"


class RStar {
public:
    RStar(double delta, std::size_t k, std::size_t maxBAStarExpansions, 
          std::size_t randomSeed = 239, std::size_t smartItersCoef = 8, double rangeLowCoef = 0.5);

    std::optional<heuristicsearch::HeuristicAlgoResult> operator()(
        const heuristicsearch::Map& map, 
        heuristicsearch::Position startPos, heuristicsearch::Position goalPos, 
        heuristicsearch::Metric heuristic,
        heuristicsearch::Metric dist = heuristicsearch::EuclideanDistance
    ) const;

protected:
    std::vector<heuristicsearch::Position> getSuccessors(const heuristicsearch::Map &map, heuristicsearch::Position pos) const;

    double delta_;
    std::size_t k_;
    std::size_t maxExpansions_;
    std::size_t randomSeed_;
    mutable std::mt19937 randomGen_;
    const std::size_t smartItersCoef_;
    const double rangeLowCoef_;
};

#endif