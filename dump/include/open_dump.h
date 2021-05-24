#ifndef DUMP_OPEN_DUMP_H
#define DUMP_OPEN_DUMP_H

#include "open.h"
#include "position.h"

#include <ostream>


class DumpInfo {
public:
    static void setOutputStream(std::ostream* os);
    static int expandedNodesCount();
    static void logAll(bool toLogAll);

public:
    static std::ostream* osPtr_;
    static int expanded_;
    static bool logAll_;
};

template <class SNode, class Priority>
class OpenSetDump : public heuristicsearch::OpenSet<SNode, Priority> {
public:
    static void setOutputStream(std::ostream* os);
    static int expandedNodesCount();
    static void logAll(bool toLogAll);

    OpenSetDump();
    void addNodeOrDecreasePriority(const SNode& snode, const Priority& p);
    std::pair<SNode, Priority> popMin();

private:
    std::ostream& os_;
};

template <class SNode, class Priority>
OpenSetDump<SNode, Priority>::OpenSetDump() : os_(*DumpInfo::osPtr_) {}

template <class SNode, class Priority>
void OpenSetDump<SNode, Priority>::addNodeOrDecreasePriority(const SNode& snode, const Priority& p) {
    if (DumpInfo::logAll_) {
        os_ << "add " << snode.row << " " << snode.col << "\n";
    }
    heuristicsearch::OpenSet<SNode, Priority>::addNodeOrDecreasePriority(snode, p);
}

template <class SNode, class Priority>
std::pair<SNode, Priority> OpenSetDump<SNode, Priority>::popMin() {
    auto [node, fval] = heuristicsearch::OpenSet<SNode, Priority>::popMin();
    if (DumpInfo::logAll_) {
        os_ << "expand " << node.row << " " << node.col << "\n";
    }
    DumpInfo::expanded_++;
    return {node, fval};
}

#endif
