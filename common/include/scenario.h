#ifndef COMMON_SCENARIO_H
#define COMMON_SCENARIO_H

#include "map.h"

#include <filesystem>
#include <vector>

namespace heuristicsearch {

class IllFormedScenarioError : public std::runtime_error {
public:
    IllFormedScenarioError(const std::string& reason);
};

class Scenario {
public:
    const Position start;
    const Position finish;

    double getBestPossibleDistance(const Map& map);

    static std::vector<Scenario> fromMovingAI(std::filesystem::path);
protected:
    Scenario(std::string mapName, 
             std::size_t mapHeight, std::size_t mapWidth, 
             const Position& from, const Position& to, double bestDistance);

    const double bestDistance_;
    const std::size_t mapHeight_;
    const std::size_t mapWidth_;
    const std::string mapName_;
};

}

#endif
