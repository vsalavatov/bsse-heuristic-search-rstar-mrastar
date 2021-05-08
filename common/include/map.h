#ifndef COMMON_MAP_H
#define COMMON_MAP_H

#include <cstddef>
#include <filesystem>
#include <vector>
#include <exception>
#include <functional>

#include "position.h"


namespace heuristicsearch {

typedef std::function<double(const Position&, const Position&)> Metric;
double EuclideanDistance(const Position&, const Position&);
double OctileDistance(const Position&, const Position&);

double pathLength(const std::vector<Position> &path, Metric m = EuclideanDistance);

class IllFormedMapError : public std::runtime_error {
public:
    IllFormedMapError(const std::string& reason);
};

class Map {
public:
    Map(const Map&) = default;
    Map(Map&&) = default;
    Map& operator=(const Map&) = default;
    Map& operator=(Map&&) = default;
    ~Map() = default;

    static Map fromMovingAI(std::filesystem::path);

    std::size_t getHeight() const;
    std::size_t getWidth() const;

    bool inBounds(Position pos) const;
    bool isTraversable(Position pos) const;
    
    std::vector<Position> getNeighbors(Position pos) const;
    
protected:
    Map();

    std::size_t height_;
    std::size_t width_;

    std::vector<std::vector<bool>> map_;  // value determines whether the cell is traversable or not
};

bool validatePath(const Map &map, const std::vector<Position> &path);

}

#endif
