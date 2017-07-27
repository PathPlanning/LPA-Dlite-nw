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

    while (open_size > 0 && (close.find(finish_node.id(explored_graph.width)) == close.end() ||
                             current_node.key < calculateKey(finish_node, finish_node, options))) {
        ++number_of_steps;
        current_node = findMin();
        if (current_node.key < calculateKey(current_node, finish_node, options)) {
            updateVertex(current_node, finish_node, options, explored_graph.width);
        } else if (current_node.g > current_node.rhs) {
            current_node.g = current_node.rhs;
            removeOpen(current_node, explored_graph.width);
            updateVertex(current_node, finish_node, options, explored_graph.width);
            for (Node node : findSuccessors(current_node, explored_graph, options)) {
                if (node.rhs > 0 && node.rhs > current_node.g + MoveCost(current_node, node)) {
                    node.rhs = current_node.g + MoveCost(current_node, node);
                    node.parent = {current_node.i, current_node.j};
                    updateVertex(node, finish_node, options, explored_graph.width);
                }
            }
        } else {
            current_node.g = std::numeric_limits<double>::infinity();
            updateVertex(current_node, finish_node, options, explored_graph.width);
            for (Node node : findSuccessors(current_node, explored_graph, options)) {
                    updateRHS(node, explored_graph, options);
                    updateVertex(node, finish_node, options, explored_graph.width);
            }
        }
        if (close.find(finish_node.id(explored_graph.width)) != close.end()) {
            finish_node = close.find(finish_node.id(explored_graph.width))->second;
        }
    }
}


SearchResult DLiteSearch::goToGoal(const Map &real_map, const EnvironmentOptions &options) {
    key_modifier = 0;
    number_of_steps = 0;
    goal.g = std::numeric_limits<double>::infinity();
    goal.rhs = 0;
    goal.parent = {goal.i, goal.j};
    updateVertex(goal, position, options, explored_graph.width);
    lppath.push_back(position);

    computePath(position, explored_graph, options);
    while (position != goal) {
        if (close.find(position.id(explored_graph.width)) == close.end() ||
            close[position.id(explored_graph.width)].g == std::numeric_limits<double>::infinity()) {
            sresult.pathfound = false;
            break;
        }
        position = close[position.id(explored_graph.width)];
        position.i = position.parent.first;
        position.j = position.parent.second;
        position = close[position.id(explored_graph.width)];
        std::cout << "Next position: " << position.i << ' ' << position.j << '\n';
        lppath.push_back(position);
        key_modifier = computeHFromCellToCell(prev, position, options);
        prev = position;
        if (exploreGraph(position, real_map, options)) {
            computePath(position, explored_graph, options);
        }
    }
    if (position == goal) {
        sresult.pathfound = true;
    }
    sresult.lppath = &lppath;
    makeSecondaryPath();
    sresult.hppath = &hppath;
    sresult.numberofsteps = number_of_steps;
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
    if (!open[node.i][node.j].is_open) {
        ++open_size;
    }
    open[node.i][node.j].is_open = true;
    open[node.i][node.j].node = node;
}

Node DLiteSearch::findMin() const {
    Node minimum;
    minimum.key = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    for (size_t i = 0; i < open.size(); ++i) {
        for (size_t j = 0; j < open[0].size(); ++j) {
            if (open[i][j].is_open) {
                minimum = std::min(minimum, open[i][j].node);
            }
        }
    }
    return minimum;
}

void DLiteSearch::deleteMin(const Node &minimum) {
    --open_size;
    open[minimum.i][minimum.j].is_open = false;
}

inline int DLiteSearch::existsOpen(const Node &node, uint_fast32_t map_width) const {
    return open[node.i][node.j].is_open;
}

inline Node DLiteSearch::findOpen(const Node &node, uint_fast32_t map_width) const {
    return open[node.i][node.j].node;
}

void DLiteSearch::removeOpen(const Node &node, uint_fast32_t map_width) {
    if (open[node.i][node.j].is_open) {
        --open_size;
    }
    open[node.i][node.j].is_open = false;
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
    return {std::min(node.g, node.rhs) + computeHFromCellToCell(node, target_node, options) + key_modifier,
            std::min(node.g, node.rhs)};
}

void DLiteSearch::updateRHS(Node &node, const Map &map, const EnvironmentOptions &options) const {
    node.rhs = std::numeric_limits<double>::infinity();
    for (Node neighbour : findSuccessors(node, map, options)) {
        if (neighbour.g + MoveCost(neighbour, node) <= node.rhs) {
            node.rhs = neighbour.g + MoveCost(neighbour, node);
            node.parent = {neighbour.i, neighbour.j};
        }
    }
}

void DLiteSearch::pointNewObstacle(unsigned i, unsigned j, const EnvironmentOptions &options,
                                   const Node &start_node) {
    Node obstacle = {i, j};
    removeOpen(obstacle, explored_graph.width);
    close.erase(obstacle.id(explored_graph.width));
    explored_graph.Grid[i][j] = CN_GC_OBS;
    for (Node node : findSuccessors(obstacle, explored_graph, options)) {
        updateRHS(node, explored_graph, options);
        updateVertex(node, position, options, explored_graph.width);
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
    open = std::vector<std::vector<open_node>>(explored_graph.height, std::vector<open_node>(explored_graph.width));
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