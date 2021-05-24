#ifndef RSTAR_RSTAR_H
#define RSTAR_RSTAR_H

#include "heuristic_algo.h"
#include "map.h"
#include "position.h"
#include "open.h"
#include "closed.h"
#include "bounded_astar.h"

#include <optional>
#include <cstddef>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <iostream>


template <class OpenSetT = heuristicsearch::OpenSet<heuristicsearch::Position, std::pair<int, double>>>
class RStar {
public:
    RStar(double delta, std::size_t k, double weight,
          double thresholdInflationFactor = 5.0, 
          std::size_t smartItersCoef = 8, double rangeLowCoef = 0.0, 
          std::size_t randomSeed = 239);

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
    double thresholdInflationFactor_;
    double weight_;
    std::size_t randomSeed_;
    mutable std::mt19937 randomGen_;
    const std::size_t smartItersCoef_;
    const double rangeLowCoef_;

    static constexpr double EPS = 1e-8;
};

template <class OpenSetT>
RStar<OpenSetT>::RStar(double delta, std::size_t k, double weight,
     double thresholdInflationFactor,
     std::size_t smartItersCoef, double rangeLowCoef,
     std::size_t randomSeed
) : delta_{delta}, k_{k}, thresholdInflationFactor_{thresholdInflationFactor}, weight_{weight},
    randomSeed_{randomSeed}, smartItersCoef_{smartItersCoef}, rangeLowCoef_{rangeLowCoef}
{}

template <class OpenSetT>
std::optional<heuristicsearch::HeuristicAlgoResult> RStar<OpenSetT>::operator()(
    const heuristicsearch::Map& map,
    heuristicsearch::Position startPos, heuristicsearch::Position goalPos,
    heuristicsearch::Metric heuristic,
    heuristicsearch::Metric dist
) const {
    using namespace heuristicsearch;

    randomGen_.seed(randomSeed_);
    auto open = OpenSetT();
    auto closed = ClosedSet<Position>();
    std::size_t totalExpansions = 0;

    std::unordered_map<Position, double> gvalue;
    std::unordered_map<Position, int> avoid;
    std::unordered_map<Position, std::pair<Position, std::vector<Position>>> parent;
    // parent -> value is {parent in Gamma, actual path on the map}
    std::unordered_map<Position, std::vector<Position>> gammaBackEdges;
    std::unordered_map<std::pair<Position, Position>, double> cLow;
    // node -> backEdges, forall n in backEdges: node in succs(n)

    auto updateState = [&](Position node) {
        if (gvalue[node] > weight_ * heuristic(startPos, node) + EPS) {
            if (!avoid.contains(node))
                avoid[node] = 1;
        }
        open.addNodeOrDecreasePriority(node, {avoid[node], gvalue[node] + weight_ * heuristic(node, goalPos)});
    };

    auto reevaluateState = [&](Position node) {
        auto bpNode = parent[node].first;
        auto thresh = cLow[{bpNode, node}] * thresholdInflationFactor_;
        if (thresh < weight_ * heuristic(bpNode, node) + EPS)
            thresh = weight_ * heuristic(bpNode, node) + EPS;
        BoundedAStarResult basr = BoundedAStar(thresh, weight_, map, bpNode, node, heuristic, dist);
        totalExpansions += basr.doneExpansions;
        if (basr.result) {
            parent[node].second = std::move(basr.result->path);
        }
        cLow[{bpNode, node}] = basr.goalCostLowerBound;
        if (parent[node].second.empty() || gvalue[bpNode] + cLow[{bpNode, node}] > weight_ * heuristic(startPos, node) + EPS) {
            Position bestPar = bpNode;
            double bestParCost = gvalue[bpNode] + cLow[{bpNode, node}];
            for (auto &backEdge : gammaBackEdges[node]) {
                auto cost = gvalue[backEdge] + cLow[{backEdge, node}];
                if (bestParCost > cost) {
                    bestParCost = cost;
                    bestPar = backEdge;
                }
            }
            if (bestPar != bpNode) {
                parent[node].first = bestPar;
                parent[node].second.clear();
            }
            avoid[node] = 1;
            bpNode = parent[node].first;
        }
        gvalue[node] = gvalue[bpNode] + cLow[{bpNode, node}];
        updateState(node);
    };

    gvalue[startPos] = 0;
    open.addNodeOrDecreasePriority(startPos, {0, weight_ * heuristic(startPos, goalPos)});
    while (!open.isEmpty()) {
        auto [node, fval] = open.popMin();

        if (parent.contains(goalPos) && !parent[goalPos].second.empty() && fval >= std::pair{avoid[goalPos], gvalue[goalPos]})
            break;

        if (node != startPos && parent[node].second.empty()) {
            reevaluateState(node);
            continue;
        }

        closed.add(node);
        auto succs = getSuccessors(map, node);
        if (dist(node, goalPos) <= delta_ && std::find(succs.begin(), succs.end(), goalPos) == succs.end()) {
            succs.push_back(goalPos);
        }
        succs.erase(std::remove_if(
            succs.begin(), succs.end(),
            [&closed](const Position &p) { return closed.contains(p); }
        ), succs.end());
        
        totalExpansions++;
        for (auto &next : succs) {
            gammaBackEdges[next].push_back(node);
            cLow[{node, next}] = heuristic(node, next);
            auto costEst = gvalue[node] + cLow[{node, next}];
            if (!gvalue.contains(next) || costEst < gvalue[next]) {
                gvalue[next] = costEst;
                parent[next].first = node;
                parent[next].second.clear();
                updateState(next);
            }
        }
    }

    if (parent[goalPos].second.empty())
        return std::nullopt;

    std::vector<Position> path{startPos};
    std::vector<Position> intermediateNodes;
    auto cur = goalPos;
    while (parent.contains(cur)) {
        intermediateNodes.push_back(cur);
        cur = parent[cur].first;
    }
    for (auto intermediateNodeIt = intermediateNodes.rbegin();
         intermediateNodeIt != intermediateNodes.rend();
         intermediateNodeIt++)
    {
        auto n = *intermediateNodeIt;
        path.insert(path.end(), parent[n].second.begin() + 1, parent[n].second.end());
    }

    return HeuristicAlgoResult{
        std::move(path),
        gvalue[goalPos],
        totalExpansions
    };
}

template <class OpenSetT>
std::vector<heuristicsearch::Position> RStar<OpenSetT>::getSuccessors(
    const heuristicsearch::Map &map, heuristicsearch::Position pos
) const {
    using namespace heuristicsearch;

    const std::size_t smartIters = k_ * smartItersCoef_;
    const double rangeLow = delta_ * rangeLowCoef_;

    std::unordered_set<Position> used;
    std::vector<Position> result;
    for (std::size_t i = 0; i < smartIters && result.size() < k_; i++) {
        const double PI = atan(1) * 4;
        double angle = std::uniform_real_distribution<>(0, 2 * PI)(randomGen_);
        double range = std::uniform_real_distribution<>(rangeLow, delta_)(randomGen_);
        double realRow = pos.row + cos(angle) * range;
        double realCol = pos.col + sin(angle) * range;
        Position next{static_cast<int>(round(realRow)), static_cast<int>(round(realCol))};
        if (map.inBounds(next) && map.isTraversable(next) && !used.contains(next)) {
            used.insert(next);
            result.push_back(next);
        }
    }

    if (result.size() < k_) {
        std::vector<Position> candidates;
        for (int dr = -delta_; dr <= delta_ && result.size() < k_; dr++) {
            for (int dc = -delta_; dc <= delta_ && result.size() < k_; dc++) {
                Position next{pos.row + dr, pos.col + dc};
                if (map.inBounds(next) && map.isTraversable(next) && !used.contains(next)) {
                    candidates.push_back(next);
                }
            }
        }
        std::shuffle(candidates.begin(), candidates.end(), randomGen_);
        std::size_t left = k_ - result.size();
        if (candidates.size() > left)
            candidates.erase(candidates.begin() + left, candidates.end());

        result.insert(result.end(), candidates.begin(), candidates.end());
    }

    return result;
}

#endif
