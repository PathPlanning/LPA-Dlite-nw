#ifndef NODE_H
#define NODE_H

#include <functional>

struct Node
{
    int     i, j;
    double  g, rhs;
    std::pair<unsigned, unsigned> parent;
    std::pair<double, double > key;

    bool operator==(const Node &other) const {
        return i == other.i && j == other.j;
    }

    bool operator!=(const Node &other) const {
        return i != other.i || j != other.j;
    }

    bool operator<(const Node &other) const {
        return key < other.key;
    }

    inline uint_least64_t id(uint_fast32_t map_width) const {
        return i * map_width + j;
    }
};

namespace std {
    template<>
    struct hash<Node> {
        size_t operator()(const Node &x) const {
            size_t seed = 0;
            seed ^= std::hash<int>()(x.i) + 0x9e3779b9
                    + (seed << 6) + (seed >> 2);
            seed ^= std::hash<int>()(x.j) + 0x9e3779b9
                    + (seed << 6) + (seed >> 2);

            return seed;
        }
    };
}
#endif
