#ifndef COMMON_CLOSED_H
#define COMMON_CLOSED_H

#include <unordered_set>


namespace heuristicsearch {

/**
 * ClosedSet
 * std::hash, == must be defined on SNode (based on position)
 */
template<class SNode>
class ClosedSet {
public:
    std::size_t size() const;
    bool isEmpty() const;
    void add(const SNode &snode);
    bool contains(const SNode &snode);
private:
    std::unordered_set<SNode> expandedNodes_;
};

template<class SNode>
std::size_t ClosedSet<SNode>::size() const {
    return expandedNodes_.size();
}

template<class SNode>
bool ClosedSet<SNode>::isEmpty() const {
    return expandedNodes_.empty();
}

template<class SNode>
void ClosedSet<SNode>::add(const SNode &snode) {
    expandedNodes_.insert(snode);
}

template<class SNode>
bool ClosedSet<SNode>::contains(const SNode &snode) {
    return expandedNodes_.contains(snode);
}

}

#endif
