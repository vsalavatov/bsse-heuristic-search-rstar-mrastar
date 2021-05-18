#include "map.h"

#include <fstream>
#include <cmath>
#include <algorithm>

namespace heuristicsearch {

/**
 * Metrics
 */

double EuclideanDistance(const Position &a, const Position &b) {
    double dr = a.row - b.row;
    double dc = a.col - b.col;
    return sqrt(dr*dr + dc*dc);
}

double OctileDistance(const Position &a, const Position &b) {
    double dr = abs(a.row - b.row);
    double dc = abs(a.col - b.col);
    double diag = std::min(dr, dc);
    double forw = std::max(dr, dc) - diag;
    return forw + diag * sqrt(2.0);
}

double pathLength(const std::vector<Position> &path, Metric m) {
    double r = 0.0;
    for (std::size_t i = 0; i + 1 < path.size(); i++) 
        r += m(path[i], path[i + 1]);
    return r;
}

/**
 * IllFormedMapError
 */
IllFormedMapError::IllFormedMapError(const std::string& reason) : std::runtime_error("map is ill-formed: " + reason) {}

/**
 * Map
 */
Map::Map() : height_{0}, width_{0}, map_{} {}

std::size_t Map::getHeight() const {
    return height_;
}
std::size_t Map::getWidth() const {
    return width_;
}

bool Map::inBounds(Position pos) const {
    return pos.row >= 0 && static_cast<std::size_t>(pos.row) < height_ && 
           pos.col >= 0 && static_cast<std::size_t>(pos.col) < width_;
}

bool Map::isTraversable(Position pos) const {
    return map_[pos.row][pos.col];
}

std::vector<Position> Map::getNeighbors(Position pos) const {
    std::vector<Position> result;
    result.reserve(8);
    for (int dr = -1; dr <= 1; dr++) {
        for (int dc = -1; dc <= 1; dc++) {
            if (dr == 0 && dc == 0) continue;
            Position np{pos.row + dr, pos.col + dc};
            if (!inBounds(np) || !isTraversable(np)) continue;
            if (abs(dr) + abs(dc) == 1) {
                result.push_back(np);
                continue;
            } 
            Position pp1{np.row, pos.col}, pp2{pos.row, np.col};
            if (isTraversable(pp1) && isTraversable(pp2)) {
                result.push_back(np);
            }
        }
    }
    return result;
}

inline static bool symIsTraversable(char c) {
    return c == '.' || c == 'G' || c == 'S' || c == 'W';
}

Map Map::fromMovingAI(std::filesystem::path path) {
    Map result;

    std::ifstream inp(path);
    std::string data;

    std::getline(inp, data);
    if (data != "type octile")
        throw IllFormedMapError("unexpected header");

    int height, width;
    std::getline(inp, data);
    if (sscanf(data.c_str(), "height %d", &height) != 1)
        throw IllFormedMapError("expected height on the second line");
    std::getline(inp, data);
    if (sscanf(data.c_str(), "width %d", &width) != 1)
        throw IllFormedMapError("expected width on the third line");

    if (height <= 0 || width <= 0)
        throw IllFormedMapError("height and/or width have illegal values");

    result.height_ = height;
    result.width_ = width;
    result.map_ = std::vector(height, std::vector<bool>(width, false));

    std::getline(inp, data);
    if (data != "map")
        throw IllFormedMapError("unexpected header on the fourth line");
    
    for (int i = 0; i < height; i++) {
        std::getline(inp, data);
        if (data.length() != static_cast<std::size_t>(width))
            throw IllFormedMapError("map's width does not match the content's width");
        for (int j = 0; j < width; j++)
            result.map_[i][j] = symIsTraversable(data[j]);
    }

    return result;
}

bool validatePath(const Map &map, const std::vector<Position> &path) {
    for (std::size_t i = 0; i < path.size(); i++) {
        if (!map.inBounds(path[i]) || !map.isTraversable(path[i]))
            return false;
    }
    for (std::size_t i = 0; i + 1 < path.size(); i++) {
        auto n = map.getNeighbors(path[i]);
        if (std::find(n.begin(), n.end(), path[i + 1]) == n.end()) 
            return false;
    }
    return true;
}

}
