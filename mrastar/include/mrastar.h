#ifndef MRASTAR_MRASTAR_H
#define MRASTAR_MRASTAR_H

#include "heuristic_algo.h"
#include "map.h"
#include "position.h"
#include "meta_algorithms.h"
#include "multi_resolution_map.h"
#include "open.h"
#include "closed.h"
#include "open_with_min_heuristic.h"

#include <vector>
#include <set>
#include <optional>


template <class ChooseQueueMethod = DynamicThompsonSamplingMethod,
          class OpenSetT = heuristicsearch::OpenSet<heuristicsearch::Position, double>>
class MRAStar {
public:
    explicit MRAStar(
        const std::set<int>& cellSizes,
        double weight,
        double suboptimalityCoef,
        ChooseQueueMethod chooseQueueMethod = ChooseQueueMethod{}
    );

    std::optional<heuristicsearch::HeuristicAlgoResult> operator()(
        const heuristicsearch::Map& map,
        heuristicsearch::Position startPos,
        heuristicsearch::Position goalPos,
        heuristicsearch::Metric heuristic,
        heuristicsearch::Metric dist = heuristicsearch::EuclideanDistance
    ) const;

protected:
    std::vector<int> cellSizes_;
    double weight_;
    double suboptimalityCoef_;
    ChooseQueueMethod chooseQueueMethod_;
};

template <class ChooseQueueMethod, class OpenSetT>
MRAStar<ChooseQueueMethod, OpenSetT>::MRAStar(
    const std::set<int>& cellSizes,
    double weight,
    double suboptimalityCoef,
    ChooseQueueMethod chooseQueueMethod
) : weight_{weight}, suboptimalityCoef_{suboptimalityCoef}, chooseQueueMethod_(std::move(chooseQueueMethod)) {

    cellSizes_.reserve(cellSizes.size() + 2);
    cellSizes_.push_back(1); // anchor
    if (!cellSizes.contains(1)) {
        cellSizes_.push_back(1);
    }
    cellSizes_.insert(cellSizes_.end(), cellSizes.begin(), cellSizes.end());
}

template <class ChooseQueueMethod, class OpenSetT>
std::optional<heuristicsearch::HeuristicAlgoResult> MRAStar<ChooseQueueMethod, OpenSetT>::operator()(
    const heuristicsearch::Map& map,
    heuristicsearch::Position startPos,
    heuristicsearch::Position goalPos,
    heuristicsearch::Metric heuristic,
    heuristicsearch::Metric dist
) const {
    using namespace heuristicsearch;

    int n = cellSizes_.size();
    auto multiResolutionMap = MultiResolutionMap(map);

    std::vector<OpenSetWithMinHeuristic<OpenSet<Position, double>>> open{cellSizes_.size()};
    std::vector<ClosedSet<Position>> closed{cellSizes_.size()};

    std::unordered_map<Position, double> gvalue;
    std::unordered_map<Position, Position> parent;

    auto priority = [&](Position node, int level) -> double {
        if (level == 0) {
            return gvalue[node] + heuristic(node, goalPos);
        }
        return gvalue[node] + weight_ * heuristic(node, goalPos);
    };
    auto expand = [&](Position node, int level) {
        for (auto& succ : multiResolutionMap.getNeighbors(node, cellSizes_[level])) {
            auto newGValue = gvalue[node] + dist(node, succ);
            if (!gvalue.contains(succ) || gvalue[succ] > newGValue) {
                gvalue[succ] = newGValue;
                parent[succ] = node;
                for (int i = 0; i < n; i++) {
                    if (multiResolutionMap.isTraversable(succ, cellSizes_[i]) && !closed[i].contains(succ)) {
                        open[i].addNodeOrDecreasePriority(succ, priority(succ, i), heuristic(succ, goalPos));
                    }
                }
            }
        }
    };

    gvalue[startPos] = 0;
    for (int i = 0; i < n; i++) {
        if (multiResolutionMap.isTraversable(startPos, cellSizes_[i])) {
            open[i].addNodeOrDecreasePriority(startPos, priority(startPos, i), heuristic(startPos, goalPos));
        }
    }

    auto queues = chooseQueueMethod_.create(open, cellSizes_);
    while (!open[1].isEmpty()) {
        int i = queues.chooseQueue();

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

        queues.update(i);
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

#endif
