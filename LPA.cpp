#include "LPA.h"
#include <vector>
#include <math.h>
#include <limits>
#include <chrono>
#include <assert.h>

LPA::LPA(float heuristic_weight, const Map &map) : hweight(heuristic_weight), current_graph(map), open_size(0) {}

double LPA::MoveCost(const Node &from, const Node &to) const {
    if (from.i != to.i && from.j != to.j)
        return M_SQRT2;
    if (from.i == to.i && from.j == to.j)
        return 0;
    return 1;
}

double LPA::computeHFromCellToCell(const Node &from, const Node &to, const EnvironmentOptions &options) const {
    switch (options.metrictype) {
        case CN_SP_MT_EUCL:
            return (sqrt((to.i - from.i) * (to.i - from.i) + (to.j - from.j) * (to.j - from.j)));
        case CN_SP_MT_DIAG:
            return (abs(abs(to.i - from.i) - abs(to.j - from.j)) +
                    sqrt(2) * (std::min(abs(to.i - from.i), abs(to.j - from.j))));
        case CN_SP_MT_MANH:
            return (abs(to.i - from.i) + abs(to.j - from.j));
        case CN_SP_MT_CHEB:
            return std::max(abs(to.i - from.i), abs(to.j - from.j));
        default:
            return 0;
    }
}


void LPA::findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options,
                         std::vector<Node> &output) const {
    output.resize(0);
    Node newNode;
    decltype(close.begin()) I;
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            if ((i != 0 || j != 0) && map.CellOnGrid(curNode.i + i, curNode.j + j) &&
                (map.CellIsTraversable(curNode.i + i, curNode.j + j))) {
                if (!options.allowdiagonal) {
                    if (i != 0 && j != 0)
                        continue;
                } else if (!options.cutcorners) {
                    if (i != 0 && j != 0)
                        if (map.CellIsObstacle(curNode.i, curNode.j + j) ||
                            map.CellIsObstacle(curNode.i + i, curNode.j))
                            continue;
                } else if (!options.allowsqueeze) {
                    if (i != 0 && j != 0)
                        if (map.CellIsObstacle(curNode.i, curNode.j + j) &&
                            map.CellIsObstacle(curNode.i + i, curNode.j))
                            continue;
                }
                newNode.i = curNode.i + i;
                newNode.j = curNode.j + j;
                I = close.find(newNode.id(map.width));
                if (I != close.end()) {
                    newNode = I->second;
                } else if (existsOpen(newNode, map.width)) {
                    newNode = findOpen(newNode, map.width);
                } else {
                    newNode.g = std::numeric_limits<double>::infinity();
                    newNode.rhs = std::numeric_limits<double>::infinity();
                }
                output.push_back(newNode);
            }
        }
    }
}

std::vector<Node> LPA::findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options) const {
    std::vector<Node> res;
    findSuccessors(curNode, map, options, res);
    return res;
}

void LPA::makeSecondaryPath() {
    std::list<Node>::const_iterator iter = lppath.begin();
    int curI, curJ, nextI, nextJ, moveI, moveJ;
    hppath.push_back(*iter);

    while (iter != --lppath.end()) {
        curI = iter->i;
        curJ = iter->j;
        ++iter;
        nextI = iter->i;
        nextJ = iter->j;
        moveI = nextI - curI;
        moveJ = nextJ - curJ;
        ++iter;
        if ((iter->i - nextI) != moveI || (iter->j - nextJ) != moveJ)
            hppath.push_back(*(--iter));
        else
            --iter;
    }
    sresult.hppath = &hppath;
}

void LPA::updateOpen(Node node, uint_fast32_t map_width) {
    if (!open[node.i][node.j].is_open) {
        ++open_size;
    }
    open[node.i][node.j].node = node;
    open[node.i][node.j].is_open = true;
}

Node LPA::findMin() const {
    Node minimum;
    minimum.key = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    for (size_t i = 0; i < open.size(); ++i) {
        for (size_t j = 0; j < open[0].size(); ++j) {
            if (open[i][j].is_open && open[i][j].node < minimum) {
                minimum = open[i][j].node;
            }
        }
    }
    return minimum;
}

inline int LPA::existsOpen(const Node &node, uint_fast32_t map_width) const {
    return open[node.i][node.j].is_open;
}

inline Node LPA::findOpen(const Node &node, uint_fast32_t map_width) const {
    return open[node.i][node.j].node;
}

void LPA::removeOpen(const Node &node, uint_fast32_t map_width) {
    if (open[node.i][node.j].is_open) {
        --open_size;
    }
    open[node.i][node.j].is_open = false;
}

void LPA::updateVertex(Node node, const Node &target_node, const EnvironmentOptions &options,
                       uint_fast32_t map_width) {
    node.key = calculateKey(node, target_node, options);
    if (node.g != node.rhs || node.rhs == std::numeric_limits<double>::infinity()) {
        close.erase(node.id(map_width));
        updateOpen(node, map_width);
    } else {
        if (existsOpen(node, map_width)) {
            removeOpen(node, map_width);
        }
        close[node.id(map_width)] = node;
    }
}

inline LPA::keytype
LPA::calculateKey(const Node &node, const Node &target_node, const EnvironmentOptions &options) const {
    return {std::min(node.g, node.rhs) + 0 * computeHFromCellToCell(node, target_node, options),
            std::min(node.g, node.rhs)};
}

double LPA::calculateRHS(const Node &node, const Map &map, const EnvironmentOptions &options) const {
    double rhs = std::numeric_limits<double>::infinity();
    for (Node neighbour : findSuccessors(node, map, options)) {
        rhs = std::min(rhs, neighbour.g + MoveCost(neighbour, node));
    }
    return rhs;
}

void LPA::pointNewObstacle(unsigned i, unsigned j, const EnvironmentOptions &options) {
    Node obstacle = {i, j};
    removeOpen(obstacle, current_graph.width);
    close.erase(obstacle.id(current_graph.width));
    current_graph.Grid[i][j] = CN_GC_OBS;
    Node goal = {current_graph.goal_i, current_graph.goal_j};
    for (Node node : findSuccessors(obstacle, current_graph, options)) {
        if (node.rhs > 0) {
            updateRHS(node, current_graph, options);
            updateVertex(node, goal, options, current_graph.width);
        }
    }
}

SearchResult LPA::startSearch(const EnvironmentOptions &options) {
    Node goal = {current_graph.goal_i, current_graph.goal_j, std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity()};
    Node start = {current_graph.start_i, current_graph.start_j, 0, 0};
    uint_least64_t number_of_steps = 0;
    if (open_size == 0 && close.size() == 0) {
        open = std::vector<std::vector<LPA::open_node>>(current_graph.height, std::vector<LPA::open_node>(current_graph.width));
        start.key = calculateKey(start, goal, options);
        updateOpen(start, current_graph.width);
    }
    int start_open_size = open_size;
    auto start_time = std::chrono::high_resolution_clock::now();
    Node current_node;
    current_node.key = {-1, -1};

    while (open_size > 0 && (close.find(goal.id(current_graph.width)) == close.end() ||
                             current_node.key < calculateKey(goal, goal, options))) {
        ++number_of_steps;
        current_node = findMin();
        if (current_node.key < calculateKey(current_node, goal, options)) {
            updateVertex(current_node, goal, options, current_graph.width);
        } else if (current_node.g > current_node.rhs) {
            current_node.g = current_node.rhs;
            removeOpen(current_node, current_graph.width);
            updateVertex(current_node, goal, options, current_graph.width);
            for (Node node : findSuccessors(current_node, current_graph, options)) {
                if (node.rhs > 0 && node.rhs > current_node.g + MoveCost(current_node, node)) {
                    node.rhs = current_node.g + MoveCost(current_node, node);
                    node.parent = {current_node.i, current_node.j};
                    updateVertex(node, goal, options, current_graph.width);
                }
            }
        } else {
            current_node.g = std::numeric_limits<double>::infinity();
            updateVertex(current_node, goal, options, current_graph.width);
            for (Node node : findSuccessors(current_node, current_graph, options)) {
                if (node.rhs > 0) {
                    updateRHS(node, current_graph, options);
                    updateVertex(node, goal, options, current_graph.width);
                }
            }
        }
        if (close.find(goal.id(current_graph.width)) != close.end()) {
            goal = close.find(goal.id(current_graph.width))->second;
        }
    }
    if (goal.g < std::numeric_limits<double>::infinity()) {
        sresult.pathfound = true;
        sresult.pathlength = goal.g;
        lppath.clear();
        hppath.clear();
        while (goal != start) {
            lppath.push_front(goal);
            goal = close[current_graph.width * goal.parent.first + goal.parent.second];
        }
        lppath.push_front(start);
    }
    auto finish_time = std::chrono::high_resolution_clock::now();
    sresult.time = std::chrono::duration_cast<std::chrono::microseconds>(finish_time - start_time).count();
    sresult.time /= 1000000;
    sresult.lppath = &lppath;
    makeSecondaryPath();
    sresult.hppath = &hppath;
    sresult.numberofsteps = number_of_steps;
    sresult.nodescreated = close.size() + (open_size - start_open_size);
    return sresult;
}

void LPA::updateRHS(Node &node, const Map &map, const EnvironmentOptions &options) const {
    node.rhs = std::numeric_limits<double>::infinity();
    for (Node neighbour : findSuccessors(node, map, options)) {
        if (neighbour.g + MoveCost(neighbour, node) <= node.rhs) {
            node.rhs = neighbour.g + MoveCost(neighbour, node);
            node.parent = {neighbour.i, neighbour.j};
        }
    }
}
