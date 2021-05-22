#ifndef COMMON_OPEN_H
#define COMMON_OPEN_H

#include <utility>
#include <unordered_map>
#include <set>

#include "hashutils.h"

namespace heuristicsearch {

/**
 * OpenSet
 * std::hash, ==, < must be defined on SNode (based on position)
 * < must be defined on Priority
 */
template<class SNode, class Priority>
class OpenSet {
public:
    std::size_t size() const;
    bool isEmpty() const;
    void addNodeOrDecreasePriority(const SNode &snode, const Priority &p);
    std::pair<SNode, Priority> popMin(); // pops a SNode with the lowest priority
    Priority minPriority();
private:
    std::unordered_map<SNode, Priority> existingElems_;
    std::set<std::pair<Priority, SNode>> heap_;
};

template<class SNode, class Priority>
std::size_t OpenSet<SNode, Priority>::size() const {
    return existingElems_.size();
}

template<class SNode, class Priority>
bool OpenSet<SNode, Priority>::isEmpty() const {
    return existingElems_.empty();
}

template<class SNode, class Priority>
void OpenSet<SNode, Priority>::addNodeOrDecreasePriority(const SNode &snode, const Priority &p) {
    if (auto it = existingElems_.find(snode); it != existingElems_.end()) {
        if (it->second <= p) return;
        heap_.erase(heap_.find({it->second, it->first}));
        it->second = p;
        heap_.insert({it->second, it->first});
    } else {
        existingElems_[snode] = p;
        heap_.insert({p, snode});
    }
}

template<class SNode, class Priority>
std::pair<SNode, Priority> OpenSet<SNode, Priority>::popMin() {
    auto elem = *heap_.begin();
    heap_.erase(heap_.begin());
    existingElems_.erase(existingElems_.find(elem.second));
    return {elem.second, elem.first};
}

template<class SNode, class Priority>
Priority OpenSet<SNode, Priority>::minPriority() {
    return heap_.begin()->first;
}

}

#endif
