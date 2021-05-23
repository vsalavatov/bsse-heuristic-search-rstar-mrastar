#include "bounded_astar.h"
#include "open.h"

using namespace heuristicsearch;


BoundedAStarResult BoundedAStar(
    double fvalThreshold,
    double weight,
    const Map& map, 
    Position startPos, Position goalPos, 
    Metric heuristic,
    Metric dist) 
{
    auto open = OpenSet<Position, double>();

    std::unordered_map<Position, double> gvalue;
    std::unordered_map<Position, Position> parent;

    auto priority = [&](Position node) -> double { return gvalue[node] + weight * heuristic(node, goalPos); };

    gvalue[startPos] = 0;
    open.addNodeOrDecreasePriority(startPos, priority(startPos));

    std::size_t doneExpansions = 0;
    while (!open.isEmpty()) {
        auto [node, fval] = open.popMin();
        
        if (gvalue.contains(goalPos) && fval > gvalue[goalPos]) 
            break;
        if (fval > fvalThreshold) {
            return BoundedAStarResult{ std::nullopt, doneExpansions, gvalue[node] + heuristic(node, goalPos) }; // uninflated!
        }
        
        doneExpansions++;
        for (auto& succ : map.getNeighbors(node)) {
            auto newGValue = gvalue[node] + dist(node, succ);
            if (!gvalue.contains(succ) || gvalue[succ] > newGValue) {
                gvalue[succ] = newGValue;
                parent[succ] = node;
                open.addNodeOrDecreasePriority(succ, priority(succ));
            }
        }
    }
    
    if (!gvalue.contains(goalPos)) {
        if (open.isEmpty()) {
            return BoundedAStarResult{ std::nullopt, doneExpansions, 1e100 };
        }
        auto [node, _] = open.popMin();
        return BoundedAStarResult{ std::nullopt, doneExpansions, gvalue[node] + heuristic(node, goalPos) }; // uninflated!
    }
    
    std::vector<Position> path;
    auto cur = goalPos;
    while (parent.contains(cur)) {
        path.push_back(cur);
        cur = parent[cur];
    }
    path.push_back(cur);
    std::reverse(path.begin(), path.end());

    return BoundedAStarResult{
        HeuristicAlgoResult{ std::move(path), gvalue[goalPos] },
        doneExpansions,
        gvalue[goalPos]
    };
}
