#include "map.h"

#include <fstream>

namespace heuristicsearch {

/**
 * IllFormatedMapError
 */
IllFormatedMapError::IllFormatedMapError(const std::string& reason) : std::runtime_error("map is ill formated: " + reason) {}

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

bool Map::inBounds(int row, int col) const {
    return row >= 0 && row < height_ && col >= 0 && col < width_;
}

bool Map::isTraversable(int row, int col) const {
    return map_[row][col];
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
        throw IllFormatedMapError("unexpected header");

    int height, width;
    std::getline(inp, data);
    if (sscanf(data.c_str(), "height %d", &height) != 1)
        throw IllFormatedMapError("expected height on the second line");
    std::getline(inp, data);
    if (sscanf(data.c_str(), "width %d", &width) != 1)
        throw IllFormatedMapError("expected width on the third line");

    if (height <= 0 || width <= 0)
        throw IllFormatedMapError("height and/or width have illegal values");

    result.height_ = height;
    result.width_ = width;
    result.map_ = std::vector(height, std::vector<bool>(width, false));

    std::getline(inp, data);
    if (data != "map")
        throw IllFormatedMapError("unexpected header on the fourth line");
    
    for (int i = 0; i < height; i++) {
        std::getline(inp, data);
        if (data.length() != width)
            throw IllFormatedMapError("map's width does not match the content's width");
        for (int j = 0; j < width; j++)
            result.map_[i][j] = symIsTraversable(data[j]);
    }

    return result;
}

};
