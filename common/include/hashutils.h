#ifndef COMMON_HASHUTILS_H
#define COMMON_HASHUTILS_H

#include <cstddef>
#include <functional>

namespace std {

template <class T>
inline void hash_combine(std::size_t& seed, const T& v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
}

template<typename T, typename S> 
struct hash<std::pair<T, S>> {
    std::size_t operator()(std::pair<T, S> const& s) const {
        std::size_t seed = 0;
        hash_combine(seed, s.first);
        hash_combine(seed, s.second);
        return seed;
    }
};

}

#endif
