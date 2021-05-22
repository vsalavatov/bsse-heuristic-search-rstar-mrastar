#include "mrastar.h"
#include "open.h"
#include "closed.h"
#include "multi_resolution_map.h"

#include <utility>

using namespace heuristicsearch;


MRAStar::MRAStar(const std::set<int>& cellSizes, double weight, double suboptimalityCoef)
    : weight_{weight}, suboptimalityCoef_{suboptimalityCoef}
{
    cellSizes_.reserve(cellSizes.size() + 2);
    cellSizes_.push_back(1); // anchor
    if (!cellSizes.contains(1)) {
        cellSizes_.push_back(1);
    }
    cellSizes_.insert(cellSizes_.end(), cellSizes.begin(), cellSizes.end());
}

std::optional<heuristicsearch::HeuristicAlgoResult>
MRAStar::operator()(const Map &map, heuristicsearch::Position startPos, heuristicsearch::Position goalPos,
                    heuristicsearch::Metric heuristic, heuristicsearch::Metric dist) const {
    int n = cellSizes_.size();
    auto multiResolutionMap = MultiResolutionMap(map);

    std::vector<OpenSet<Position, double>> open{cellSizes_.size()};
    std::vector<ClosedSet<Position>> closed{cellSizes_.size()};

    std::unordered_map<Position, double> gvalue;
    std::unordered_map<Position, Position> parent;

    auto priority = [&](Position node, int level) -> double {
        if (level == 0) {
            return gvalue[node] + heuristic(node, goalPos);
        }
        return gvalue[node] + weight_ * heuristic(node, goalPos);
    };
    auto chooseQueue = [&]() -> int {
        static int next = 0;
        while (open[next].isEmpty()) {
            next = (next + 1) % n;
        }
        int result = next;
        next = (next + 1) % n;
        return result;
    };
    auto expand = [&](Position node, int level) {
        for (auto& succ : multiResolutionMap.getNeighbors(node, cellSizes_[level])) {
            auto newGValue = gvalue[node] + dist(node, succ);
            if (!gvalue.contains(succ) || gvalue[succ] > newGValue) {
                gvalue[succ] = newGValue;
                parent[succ] = node;
                for (int i = 0; i < n; i++) {
                    if (multiResolutionMap.isTraversable(succ, cellSizes_[i]) && !closed[i].contains(succ)) {
                        open[i].addNodeOrDecreasePriority(succ, priority(succ, i));
                    }
                }
            }
        }
    };

    gvalue[startPos] = 0;
    for (int i = 0; i < n; i++) {
        if (multiResolutionMap.isTraversable(startPos, cellSizes_[i])) {
            open[i].addNodeOrDecreasePriority(startPos, priority(startPos, i));
        }
    }

    while (!open[0].isEmpty()) {
        int i = chooseQueue();

        int toExpand = 0;
        if (open[i].minPriority() <= suboptimalityCoef_ * open[0].minPriority()) {
            if (gvalue.contains(goalPos) && gvalue[goalPos] <= open[i].minPriority()) {
                break;
            }
            toExpand = i;
        } else if (gvalue.contains(goalPos) && gvalue[goalPos] <= suboptimalityCoef_ * open[i].minPriority()) {
            break;
        }
        auto [node, fval] = open[toExpand].popMin();
        expand(node, toExpand);
        closed[toExpand].add(node);
    }

    if (!gvalue.contains(goalPos)) return std::nullopt;

    std::vector<Position> path;
    auto cur = goalPos;
    auto prev = cur;
    while (true) {
        if (prev == cur) {
            path.push_back(cur);
        } else {
            int len = std::max(std::abs(cur.row - prev.row), std::abs(cur.col - prev.col));
            int dr = (cur.row - prev.row) / len;
            int dc = (cur.col - prev.col) / len;
            for (int d = 1; d <= len; d++) {
                path.push_back({prev.row + dr * d, prev.col + dc * d});
            }
        }
        prev = cur;
        if (!parent.contains(cur)) {
            break;
        }
        cur = parent[cur];
    }
    std::reverse(path.begin(), path.end());

    return HeuristicAlgoResult{
        std::move(path),
        gvalue[goalPos]
    };
}
