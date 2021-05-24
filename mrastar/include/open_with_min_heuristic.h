#ifndef MRASTAR_OPEN_WITH_MIN_HEURISTIC_H
#define MRASTAR_OPEN_WITH_MIN_HEURISTIC_H

#include <limits>

template <class OpenSetT>
class OpenSetWithMinHeuristic : public OpenSetT {
public:
    using NodeType = typename OpenSetT::NodeType;
    using PriorityType = typename OpenSetT::PriorityType;

    void addNodeOrDecreasePriority(const NodeType& snode, const PriorityType& p, const PriorityType& h) {
        openH.addNodeOrDecreasePriority(snode, h);
        OpenSetT::addNodeOrDecreasePriority(snode, p);
    }

    PriorityType minHeuristicValue() const {
        if (openH.isEmpty()) {
            return std::numeric_limits<PriorityType>::infinity();
        }
        return openH.minPriority();
    }

    void deleteNode(const NodeType &snode) {
        openH.deleteNode(snode);
        OpenSetT::deleteMode(snode);
    }

    std::pair<NodeType, PriorityType> popMin() {
        auto [node, fval] = OpenSetT::popMin();
        openH.deleteNode(node);
        return {node, fval};
    }

private:
    heuristicsearch::OpenSet<NodeType, PriorityType> openH;
};

#endif
