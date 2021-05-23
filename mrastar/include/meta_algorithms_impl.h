#ifndef MRASTAR_META_ALGORITHMS_IMPL_H
#define MRASTAR_META_ALGORITHMS_IMPL_H

#include "open.h"
#include "position.h"

#include <boost/random.hpp>
#include <boost/random/beta_distribution.hpp>
#include <utility>
#include <vector>
#include <utility>


template <class OpenSetT>
class MultiHeuristicMetaMethodImpl {
public:
    virtual void update(int queueIndex) = 0;
    virtual int chooseQueue() = 0;

protected:
    explicit MultiHeuristicMetaMethodImpl(const std::vector<OpenSetT>& queues) : queues_(queues) {}

    const std::vector<OpenSetT>& queues_;
};

template <class OpenSetT>
class RoundRobinMethodImpl : public MultiHeuristicMetaMethodImpl<OpenSetT> {
private:
    using Base = MultiHeuristicMetaMethodImpl<OpenSetT>;
public:
    explicit RoundRobinMethodImpl(const std::vector<OpenSetT>& queues)
            : MultiHeuristicMetaMethodImpl<OpenSetT>(queues) {}

    void update(int) override {}

    int chooseQueue() override {
        do {
            index_ = 1 + index_ % (Base::queues_.size() - 1);
        } while (Base::queues_[index_].isEmpty());

        return index_;
    }

private:
    int index_ = 0;
};

template <class OpenSetT>
class MetaAStarMethodImpl : public MultiHeuristicMetaMethodImpl<OpenSetT> {
private:
    using Base = MultiHeuristicMetaMethodImpl<OpenSetT>;
public:
    MetaAStarMethodImpl(const std::vector<OpenSetT>& queues, std::vector<int> cellSizes)
            : MultiHeuristicMetaMethodImpl<OpenSetT>{queues}, cellSizes_{std::move(cellSizes)} {
        metaGValue_.resize(cellSizes_.size());
        metaHeuristic_.resize(cellSizes_.size());
        metaPriority_.resize(cellSizes_.size());

        for (std::size_t i = 1; i < cellSizes_.size(); i++) {
            if (!Base::queues_[i].isEmpty()) {
                metaPriority_[i] = metaHeuristic_[i] = Base::queues_[i].minHeuristicValue() / cellSizes_[i];
            }
        }
    }

    void update(int queueIndex) override {
        metaGValue_[queueIndex] += 1;
        if (Base::queues_[queueIndex].isEmpty()) {
            return;
        }
        metaHeuristic_[queueIndex] = Base::queues_[queueIndex].minHeuristicValue() / cellSizes_[queueIndex];
        metaPriority_[queueIndex] = metaGValue_[queueIndex] + metaHeuristic_[queueIndex];
    }

    int chooseQueue() override {
        double minPriority = std::numeric_limits<double>::infinity();
        int result = 0;
        for (std::size_t i = 1; i < Base::queues_.size(); i++) {
            if (!Base::queues_[i].isEmpty() && metaPriority_[i] < minPriority) {
                minPriority = metaPriority_[i];
                result = i;
            }
        }
        return result;
    }

private:
    std::vector<int> cellSizes_;

    std::vector<double> metaGValue_;
    std::vector<double> metaHeuristic_;
    std::vector<double> metaPriority_;
};

template <class OpenSetT>
class DynamicThompsonSamplingMethodImpl : public MultiHeuristicMetaMethodImpl<OpenSetT> {
private:
    using Base = MultiHeuristicMetaMethodImpl<OpenSetT>;
public:
    DynamicThompsonSamplingMethodImpl(
            const std::vector<OpenSetT>& queues,
            std::vector<int> cellSizes,
            double historyCoef,
            std::size_t randomSeed
    ) : MultiHeuristicMetaMethodImpl<OpenSetT>{queues}, cellSizes_{std::move(cellSizes)}, historyCoef_{historyCoef} {
        randomGen_.seed(randomSeed);

        alpha_.resize(cellSizes_.size());
        beta_.resize(cellSizes_.size());
        bestH_.resize(cellSizes_.size());

        for (std::size_t i = 1; i < cellSizes_.size(); i++) {
            bestH_[i] = Base::queues_[i].minHeuristicValue();
            alpha_[i] = 1;
            beta_[i] = 1;
        }
    }

    void update(int queueIndex) override {
        auto newBestH = Base::queues_[queueIndex].minHeuristicValue();
        if (newBestH < bestH_[queueIndex]) {
            bestH_[queueIndex] = newBestH;
            alpha_[queueIndex] += 1;
        } else {
            beta_[queueIndex] += 1;
        }
        if (alpha_[queueIndex] + beta_[queueIndex] > historyCoef_) {
            auto sum = alpha_[queueIndex] + beta_[queueIndex];
            alpha_[queueIndex] *= historyCoef_ / sum;
            beta_[queueIndex] *= historyCoef_ / sum;
        }
    }

    int chooseQueue() override {
        double maxR = -std::numeric_limits<double>::infinity();
        int result = 0;
        for (std::size_t i = 1; i < Base::queues_.size(); i++) {
            if (Base::queues_[i].isEmpty()) {
                continue;
            }

            auto betaDist = boost::random::beta_distribution(alpha_[i], beta_[i]);
            boost::variate_generator<boost::mt19937&, decltype(betaDist)> varBeta{randomGen_, betaDist};

            auto r = varBeta();
            if (r > maxR) {
                maxR = r;
                result = i;
            }
        }
        return result;
    }

private:
    std::vector<int> cellSizes_;
    double historyCoef_;

    boost::mt19937 randomGen_;

    std::vector<double> alpha_;
    std::vector<double> beta_;
    std::vector<double> bestH_;
};

#endif
