#include "open_dump.h"

std::ostream* DumpInfo::osPtr_ = nullptr;
int DumpInfo::expanded_ = 0;
bool DumpInfo::logAll_ = false;

void DumpInfo::setOutputStream(std::ostream* os) {
    osPtr_ = os;
}

int DumpInfo::expandedNodesCount() {
    return expanded_;
}

void DumpInfo::logAll(bool toLogAll) {
    logAll_ = toLogAll;
}