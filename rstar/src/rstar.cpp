#include "rstar.h"
#include "open.h"
#include "closed.h"
#include "bounded_astar.h"
#include "heuristic_algo.h"

#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>

using namespace heuristicsearch;


RStar::RStar(double delta, std::size_t k, std::size_t maxBAStarExpansions, 
             std::size_t randomSeed, std::size_t smartItersCoef, double rangeLowCoef) 
    : delta_{delta}, k_{k}, maxExpansions_{maxBAStarExpansions}, 
      randomSeed_{randomSeed}, smartItersCoef_{smartItersCoef}, rangeLowCoef_{rangeLowCoef}
{}

std::optional<HeuristicAlgoResult> RStar::operator()(
    const Map& map, 
    Position startPos, Position goalPos, 
    Metric heuristic,
    Metric dist
) const {
    randomGen_.seed(randomSeed_);
    auto open = OpenSet<Position, std::pair<int, double>>();
    auto closed = ClosedSet<Position>();

    std::unordered_map<Position, double> gvalue;
    std::unordered_set<Position> avoid;
    std::unordered_map<Position, std::pair<Position, std::vector<Position>>> parent; 
    // parent -> value is {parent in Gamma, actual path on the map}
    std::unordered_map<Position, std::vector<Position>> gammaBackEdges;
    std::unordered_map<std::pair<Position, Position>, double> cLow;
    // node -> backEdges, forall n in backEdges: node in succs(n) 
    
    auto updateState = [&](Position node) {
        int priority = (gvalue[node] > heuristic(startPos, node) || (avoid.contains(node) && parent[node].second.size() == 0)) ? 1 : 0;        
        open.addNodeOrDecreasePriority(node, {priority, gvalue[node] + heuristic(node, goalPos)});
    };

    auto reevaluateState = [&](Position node) {
        auto bpNode = parent[node].first;
        BoundedAStarResult basr = BoundedAStar(maxExpansions_, map, bpNode, node, heuristic, dist);
        if (basr.result) {
            parent[node].second = std::move(basr.result->path);
        }
        cLow[{bpNode, node}] = basr.goalCostLowerBound;
        if (parent[node].second.size() == 0 || gvalue[bpNode] + cLow[{bpNode, node}] > heuristic(startPos, node)) {
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
            avoid.insert(node);
            bpNode = parent[node].first;
        }
        gvalue[node] = gvalue[bpNode] + cLow[{bpNode, node}];
        updateState(node);
    };

    gvalue[startPos] = 0;
    open.addNodeOrDecreasePriority(startPos, {0, heuristic(startPos, goalPos)});
    while (!open.isEmpty()) {
        auto [node, fval] = open.popMin();

        if (gvalue.contains(goalPos) && fval > std::pair{static_cast<int>(avoid.contains(goalPos)), gvalue[goalPos]}) 
            break;
        
        if (node != startPos && parent[node].second.size() == 0) {
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

        for (auto &next : succs) {
            gammaBackEdges[next].push_back(node);
            cLow[{node, next}] = heuristic(node, next);
            auto costEst = gvalue[node] + cLow[{node, next}];
            if (!gvalue.contains(next) || costEst < gvalue[next]) {
                gvalue[next] = costEst;
                parent[next].first = node;
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
        gvalue[goalPos]
    };
}

std::vector<Position> RStar::getSuccessors(const heuristicsearch::Map &map, heuristicsearch::Position pos) const {
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
