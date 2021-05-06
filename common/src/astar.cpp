#include "astar.h"

namespace heuristicsearch {

std::optional<HeuristicAlgoResult> AStar(
    const Map& map, 
    Position startPos, Position goalPos, 
    std::function<double(Position, Position)> heuristic)
{
    auto open = OpenSet<Position, double>();

    std::unordered_map<Position, double> gvalue;
    std::unordered_map<Position, Position> parent;

    auto priority = [&](Position node) -> double { return gvalue[node] + heuristic(node, goalPos); };

    gvalue[startPos] = 0;
    open.addNodeOrDecreasePriority(startPos, priority(startPos));

    while (!open.isEmpty()) {
        auto [node, fval] = open.popMin();
        
        if (gvalue.contains(goalPos) && fval > gvalue[goalPos]) 
            break;
        
        for (auto& succ : map.getNeighbors(node)) {
            auto newGValue = gvalue[node] + EuclideanDistance(node, succ);
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
        gvalue[goalPos]
    };
}

} // namespace heuristicsearch
