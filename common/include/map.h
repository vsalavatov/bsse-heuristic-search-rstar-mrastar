#ifndef COMMON_MAP_H
#define COMMON_MAP_H

#include <cstddef>
#include <filesystem>
#include <vector>
#include <exception>
#include <functional>

namespace heuristicsearch {

struct Position {
    int row;
    int col;
};

typedef std::function<double(const Position&, const Position&)> Metric;
double EuclideanDistance(const Position&, const Position&);
double OctileDistance(const Position&, const Position&);

class IllFormatedMapError : public std::runtime_error {
public:
    IllFormatedMapError(const std::string& reason);
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
    
protected:
    Map();

    std::size_t height_;
    std::size_t width_;

    std::vector<std::vector<bool>> map_;  // value determines whether the cell is traversable or not
};

};

#endif
