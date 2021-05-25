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

template <class OpenSetT = heuristicsearch::OpenSet<heuristicsearch::Position, double>>
std::optional<HeuristicAlgoResult> AStar(
    const Map& map, 
    Position startPos, Position goalPos, 
    Metric heuristic,
    Metric dist = EuclideanDistance
) {
    using namespace heuristicsearch;

    auto open = OpenSetT();
    std::size_t expansions = 0;

    std::unordered_map<Position, double> gvalue;
    std::unordered_map<Position, Position> parent;

    auto priority = [&](Position node) -> double { return gvalue[node] + heuristic(node, goalPos); };

    gvalue[startPos] = 0;
    open.addNodeOrDecreasePriority(startPos, priority(startPos));
    
    while (!open.isEmpty()) {
        auto [node, fval] = open.popMin();

        if (gvalue.contains(goalPos) && fval > gvalue[goalPos])
            break;

        expansions++;
        for (auto& succ : map.getNeighbors(node)) {
            auto newGValue = gvalue[node] + dist(node, succ);
            if (!gvalue.contains(succ) || gvalue[succ] > newGValue) {
                gvalue[succ] = newGValue;
                parent[succ] = node;
                open.addNodeOrDecreasePriority(succ, priority(succ));
            }
        }
    }

    if (!gvalue.contains(goalPos)) return std::nullopt;

    std::vector<Position> path;
    auto cur = goalPos;
    while (parent.contains(cur)) {
        path.push_back(cur);
        cur = parent[cur];
    }
    path.push_back(cur);
    std::reverse(path.begin(), path.end());

    return HeuristicAlgoResult{
            std::move(path),
            gvalue[goalPos],
            expansions
    };
}

}

#endif
