#include "bounded_astar.h"
#include "open.h"

using namespace heuristicsearch;


BoundedAStarResult BoundedAStar(
    std::size_t maxExpansions,
    const Map& map, 
    Position startPos, Position goalPos, 
    Metric heuristic,
    Metric dist) 
{
    auto open = OpenSet<Position, double>();

    std::unordered_map<Position, double> gvalue;
    std::unordered_map<Position, Position> parent;

    auto priority = [&](Position node) -> double { return gvalue[node] + heuristic(node, goalPos); };

    gvalue[startPos] = 0;
    open.addNodeOrDecreasePriority(startPos, priority(startPos));

    std::size_t doneExpansions = 0;
    double lastfval = 1e18;
    while (!open.isEmpty() && doneExpansions < maxExpansions) {
        auto [node, fval] = open.popMin();
        lastfval = fval;
        
        if (gvalue.contains(goalPos) && fval > gvalue[goalPos]) 
            break;
        
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
        if (open.isEmpty())
            return BoundedAStarResult{ std::nullopt, doneExpansions, 1e18 };
        return BoundedAStarResult{ std::nullopt, doneExpansions, lastfval };
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
