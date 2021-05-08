#include "scenario.h"

#include <fstream>
#include <sstream>
#include <cmath>

namespace heuristicsearch {

IllFormedScenarioError::IllFormedScenarioError(const std::string &reason) : std::runtime_error("scenario is ill-formed: " + reason)
{}

Scenario::Scenario(std::string mapName, 
                   std::size_t mapHeight, std::size_t mapWidth, 
                   const Position& from, const Position& to, double bestDistance)
    : start{from}, finish{to}, bestDistance_{bestDistance}, mapHeight_{mapHeight}, mapWidth_{mapWidth}, mapName_{std::move(mapName)}
{}

double Scenario::getBestPossibleDistance(const Map& map) {
    // see https://movingai.com/benchmarks/formats.html
    double ratioW = static_cast<double>(map.getWidth()) / mapWidth_;
    double ratioH = static_cast<double>(map.getHeight()) / mapHeight_;
    if (fabs(ratioW - ratioH) > 1e-9)
        throw IllFormedScenarioError("scenario's map size cannot be scaled to actual map's size");
    return bestDistance_ * ratioW;
}

std::vector<Scenario> Scenario::fromMovingAI(std::filesystem::path path) {
    std::ifstream inp(path);
    std::string data;

    std::getline(inp, data);
    if (!data.starts_with("version"))
        throw IllFormedMapError("unexpected header");

    std::vector<Scenario> scenarios;
    while (std::getline(inp, data) && data.length() > 0) {
        std::stringstream reader{data};
        int bucket;
        std::size_t w, h;
        std::string mapname;
        Position from, to;
        double dist;
        if (!(reader >> bucket)) throw IllFormedScenarioError("expected bucket");
        if (!(reader >> mapname)) throw IllFormedScenarioError("expected map name");
        if (!(reader >> w)) throw IllFormedScenarioError("expected map width");
        if (!(reader >> h)) throw IllFormedScenarioError("expected map height");
        if (!(reader >> from.col)) throw IllFormedScenarioError("expected start.x");
        if (!(reader >> from.row)) throw IllFormedScenarioError("expected start.y");
        if (!(reader >> to.col)) throw IllFormedScenarioError("expected goal.x");
        if (!(reader >> to.row)) throw IllFormedScenarioError("expected goal.y");
        if (!(reader >> dist)) throw IllFormedScenarioError("expected best possible distance");
        scenarios.push_back(Scenario(mapname, h, w, from, to, dist));
    }

    return scenarios;
}

}
