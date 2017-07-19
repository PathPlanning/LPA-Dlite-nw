#include "dsearch.h"
#include <vector>
#include <math.h>
#include <limits>
#include <chrono>
#include <assert.h>

DLiteSearch::DLiteSearch() {
    hweight = 1;
    open_size = 0;
}

DLiteSearch::DLiteSearch(float heuristic_weigh, unsigned map_width, unsigned map_height, int view_radius)
        : hweight(heuristic_weigh), explored_graph(map_width, map_height, {0, 0}, {0, 0}) {
    this->view_radius = view_radius;
    graph_has_changed = false;
}

double DLiteSearch::MoveCost(const Node &from, const Node &to) const {
    if (from.i != to.i && from.j != to.j)
        return M_SQRT2;
    if (from.i == to.i && from.j == to.j)
        return 0;
    return 1;
}

double DLiteSearch::computeHFromCellToCell(const Node &from, const Node &to, const EnvironmentOptions &options) const {
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

void DLiteSearch::computePath(Node finish_node, const Map &map, const EnvironmentOptions &options) {
    Node current_node;
    current_node.key = {-1, -1};
    std::vector<Node> successors;
    successors.reserve(9);
    while (open_size > 0 && (close.find(finish_node.id(map.width)) == close.end() ||
                             current_node.key < calculateKey(finish_node, finish_node, options))) {
        ++number_of_steps;
        current_node = findMin();
        std::cout << "Expanding " << current_node.i << ' ' << current_node.j << " g = " << current_node.g << " rhs = "
                  << current_node.rhs << '\n';
        if (current_node.key < calculateKey(current_node, finish_node, options)) {
            current_node.key = calculateKey(current_node, finish_node, options);
            updateOpen(current_node, map.width);
        } else {
            findSuccessors(current_node, map, options, successors);
            if (current_node.g >= current_node.rhs) {
                deleteMin(current_node);
                current_node.g = current_node.rhs;
                updateVertex(current_node, finish_node, options, map.width);
                for (Node node : successors) {
                    node.rhs = std::min(node.rhs, current_node.g + MoveCost(current_node, node));
                    updateVertex(node, finish_node, options, map.width);
                }
            } else {
                current_node.g = std::numeric_limits<double>::infinity();
                updateVertex(current_node, finish_node, options, map.width);
                successors.push_back(current_node);
                for (Node node : successors) {
                    if (node.rhs > 0) {
                        node.rhs = calculateRHS(node, map, options);
                    }
                    updateVertex(node, finish_node, options, map.width);
                }
            }
        }
        auto I = close.find(finish_node.id(map.width));
        if (I != close.end()) {
            finish_node = I->second;
        }
    }
}

SearchResult DLiteSearch::goToGoal(const Map &real_map, const EnvironmentOptions &options) {
    number_of_steps = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    position.g = std::numeric_limits<double>::infinity();
    position.rhs = std::numeric_limits<double>::infinity();
    lppath.push_back(position);
    goal.key = calculateKey(goal, position, options);
    updateOpen(goal, explored_graph.width);
    computePath(position, explored_graph, options);
    std::vector<Node> neighbours(8);

    while (position != goal) {
        auto I = close.find(position.id(explored_graph.width));
        if (I == close.end() || I->second.g == std::numeric_limits<double>::infinity()) {
            break;
        } else {
            position = I->second;
        }

        makeStep(options);

        if (exploreGraph(position, real_map, options)) {
            key_modifier += computeHFromCellToCell(prev, position, options);
            prev = position;
            reorderOpen();
            computePath(position, explored_graph, options);
        }
    }
    auto finish_time = std::chrono::high_resolution_clock::now();
    sresult.time = (double) (std::chrono::duration_cast<std::chrono::microseconds>(finish_time - start_time).count()) /
                   1000000;
    if (position == goal) {
        sresult.pathfound = true;
    }
    sresult.lppath = &lppath;
    sresult.pathlength = 0;
    for (auto it = lppath.begin(), it_prev = lppath.begin(); it != lppath.end(); ++it) {
        if (it != lppath.begin()) {
            sresult.pathlength += MoveCost(*it_prev, *it);
            ++it_prev;
        }
    }
    makeSecondaryPath();
    sresult.numberofsteps = number_of_steps;
    sresult.nodescreated = close.size() + open_size;
    return sresult;
}

void DLiteSearch::findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options,
                                 std::vector<Node> &output) const {
    output.resize(0);
    Node newNode;
    decltype(close.begin()) I;
    for (int i = -1; i <= +1; ++i) {
        for (int j = -1; j <= +1; ++j) {
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

std::vector<Node> DLiteSearch::findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options) const {
    std::vector<Node> res;
    findSuccessors(curNode, map, options, res);
    return res;
}

void DLiteSearch::makeSecondaryPath() {
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

void DLiteSearch::updateOpen(Node node, uint_fast32_t map_width) {
    auto node_id = node.id(map_width);
    if (open[node.i].find(node_id) == open[node.i].end()) {
        ++open_size;
    }
    open[node.i][node_id] = node;

    if (open[node.i].size() == 1) {
        cluster_minimums[node.i] = node_id;
    } else {
        Node min = open[node.i][cluster_minimums[node.i]];
        if (node < min) {
            cluster_minimums[node.i] = node_id;
        }
    }
}

Node DLiteSearch::findMin() const {
    Node minimum;
    minimum.key = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    for (size_t i = 0; i < open.size(); ++i) {
        if (!open[i].empty()) {
            if (open[i].at(cluster_minimums[i]) < minimum) {
                minimum = open[i].at(cluster_minimums[i]);
            }
        }
    }
    return minimum;
}

void DLiteSearch::deleteMin(const Node &minimum) {
    open[minimum.i].erase(cluster_minimums[minimum.i]);
    --open_size;
    if (!open[minimum.i].empty()) {
        DLiteSearch::keytype min_key = {std::numeric_limits<double>::infinity(),
                                        std::numeric_limits<double>::infinity()};
        for (auto it = open[minimum.i].begin(); it != open[minimum.i].end(); ++it) {
            if (it->second.key < min_key) {
                min_key = it->second.key;
                cluster_minimums[minimum.i] = it->first;
            }
        }
    }
}

inline int DLiteSearch::existsOpen(const Node &node, uint_fast32_t map_width) const {
    return (open[node.i].find(node.id(map_width)) != open[node.i].end());
}

inline Node DLiteSearch::findOpen(const Node &node, uint_fast32_t map_width) const {
    return open[node.i].at(node.id(map_width));
}

void DLiteSearch::removeOpen(const Node &node, uint_fast32_t map_width) {
    if (cluster_minimums[node.i] == node.id(map_width)) {
        deleteMin(node);
    } else {
        open[node.i].erase(node.id(map_width));
        --open_size;
    }
}

void DLiteSearch::reorderOpen() {
    for (size_t i = 0; i < open.size(); ++i) {
        if (!open[i].empty()) {
            DLiteSearch::keytype min_key = {std::numeric_limits<double>::infinity(),
                                            std::numeric_limits<double>::infinity()};
            for (auto it = open[i].begin(); it != open[i].end(); ++it) {
                if (it->second.key < min_key) {
                    min_key = it->second.key;
                    cluster_minimums[i] = it->first;
                }
            }
        }
    }
}

void DLiteSearch::updateVertex(Node &node, const Node &target_node, const EnvironmentOptions &options,
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

inline DLiteSearch::keytype
DLiteSearch::calculateKey(const Node &node, const Node &target_node, const EnvironmentOptions &options) const {
    return {std::min(node.g, node.rhs) + computeHFromCellToCell(node, target_node, options),
            std::min(node.g, node.rhs)};
}

double DLiteSearch::calculateRHS(const Node &node, const Map &map, const EnvironmentOptions &options) const {
    double rhs = std::numeric_limits<double>::infinity();
    for (Node neighbour : findSuccessors(node, map, options)) {
        rhs = std::min(rhs, neighbour.g + MoveCost(neighbour, node));
    }
    return rhs;
}

void DLiteSearch::pointNewObstacle(unsigned i, unsigned j, const EnvironmentOptions &options,
                                   const Node &start_node) {
    explored_graph.Grid[i][j] = CN_GC_OBS;
    Node obstacle = {i, j, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    auto I = close.find(obstacle.id(explored_graph.width));
    if (I != close.end()) {
        obstacle = I->second;
        close.erase(obstacle.id(explored_graph.width));
    } else if (existsOpen(obstacle, explored_graph.width)) {
        obstacle = findOpen(obstacle, explored_graph.width);
        removeOpen(obstacle, explored_graph.width);
    }
    for (Node neighbour : findSuccessors(obstacle, explored_graph, options)) {
        if (neighbour.rhs < std::numeric_limits<double>::infinity() && neighbour.rhs > 0) {
            neighbour.rhs = calculateRHS(neighbour, explored_graph, options);
            updateVertex(neighbour, start_node, options, explored_graph.width);
        }
    }
}

int DLiteSearch::exploreGraph(const Node &position, const Map &real_map,
                              const EnvironmentOptions &options) {
    int changed = 0;
    std::vector<Node> obstacles;
    for (int i = -view_radius; i <= view_radius; ++i) {
        for (int j = -view_radius; j <= view_radius; ++j) {
            if (!(i == 0 && j == 0) && real_map.CellOnGrid(position.i + i, position.j + j) &&
                real_map.CellIsObstacle(position.i + i, position.j + j)) {
                pointNewObstacle(position.i + i, position.j + j, options, position);
                changed = 1;
            }
        }
    }

    return changed;
}

void DLiteSearch::changeGoal(Node new_goal) {
    key_modifier = 0;
    open_size = 0;
    open = std::vector<DLiteSearch::open_claster_t>(explored_graph.height);
    cluster_minimums.resize(explored_graph.height);
    close.clear();

    new_goal.g = 0;
    new_goal.rhs = 0;
    goal = new_goal;
}

void DLiteSearch::setStartPosition(const Node &start) {
    position = start;
    prev = start;
}

void DLiteSearch::makeStep(const EnvironmentOptions &options) {
    for (Node node : findSuccessors(position, explored_graph, options)) {
        auto I = close.find(node.id(explored_graph.width));
        if (I != close.end()) {
            node = I->second;
            if (node.g + MoveCost(node, prev) <= position.g + MoveCost(position, prev) || node == goal) {
                position = node;
            }
        }
    }
    std::cout << "Next position: " << position.i << ' ' << position.j << '\n';
    lppath.push_back(position);
}