#ifndef MRASTAR_META_ALGORITHMS_H
#define MRASTAR_META_ALGORITHMS_H

#include "meta_algorithms_impl.h"


class RoundRobinMethod {
public:
    template <class OpenSetT>
    RoundRobinMethodImpl<OpenSetT> create(const std::vector<OpenSetT>& queues, std::vector<int>) const {
        return RoundRobinMethodImpl<OpenSetT>{queues};
    }
};

class MetaAStarMethod {
public:
    template <class OpenSetT>
    MetaAStarMethodImpl<OpenSetT> create(const std::vector<OpenSetT>& queues, std::vector<int> cellSizes) const {
        return MetaAStarMethodImpl<OpenSetT>{queues, std::move(cellSizes)};
    }
};

class DynamicThompsonSamplingMethod {
public:
    explicit DynamicThompsonSamplingMethod(double historyCoef = 10, std::size_t randomSeed = 30)
        : historyCoef_{historyCoef}, randomSeed_{randomSeed} {}

    template <class OpenSetT>
    DynamicThompsonSamplingMethodImpl<OpenSetT> create(const std::vector<OpenSetT>& queues,
                                                       std::vector<int> cellSizes) const {
        return DynamicThompsonSamplingMethodImpl<OpenSetT>{queues, std::move(cellSizes), historyCoef_, randomSeed_};
    }

private:
    double historyCoef_;
    std::size_t randomSeed_;
};

#endif
