#include "map.h"

#include <fstream>
#include <cmath>
#include <algorithm>

namespace heuristicsearch {

/**
 * Position
 */

bool operator==(Position a, Position b) {
    return a.row == b.row && a.col == b.col;
}

/**
 * Metrics
 */

double EuclideanDistance(const Position &a, const Position &b) {
    double dr = a.row - b.row;
    double dc = a.col - b.col;
    return sqrt(dr*dr + dc*dc);
}

double OctileDistance(const Position &a, const Position &b) {
    double dr = fabs(a.row - b.row);
    double dc = fabs(a.col - b.col);
    double forw = std::min(dr, dc);
    double diag = std::max(dr, dc) - forw;
    return forw + sqrt(diag*diag);
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

}
