//
// Created by dmitry on 19.07.17.
//

#ifndef D_LITE_LPA_H
#define D_LITE_LPA_H

#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include "node.h"
#include <list>
#include <unordered_map>

class LPA {
    typedef std::pair<double, double> keytype;
    typedef std::unordered_map<uint_least64_t, Node> open_cluster_t;

protected:
    Node findMin() const;

    LPA::keytype topOpenKey() const;

    void deleteMin(const Node &minimum);

    void updateOpen(Node node, uint_fast32_t map_width);

    int existsOpen(const Node &node, uint_fast32_t map_width) const;

    Node findOpen(const Node &node, uint_fast32_t map_width) const;

    void removeOpen(const Node &node, uint_fast32_t map_width);

    double MoveCost(const Node &from, const Node &to) const;

    double computeHFromCellToCell(const Node &from, const Node &to, const EnvironmentOptions &options) const;

    void
    findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options, std::vector<Node> &output) const;

    std::vector<Node> findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options) const;

    void makeSecondaryPath();

    keytype calculateKey(const Node &node, const Node &target_node, const EnvironmentOptions &options) const;

    void updateVertex(Node node, const Node &target_node, const EnvironmentOptions &options, uint_fast32_t map_width);

    void updateRHS(Node &node, const Map &map, const EnvironmentOptions &options) const;
public:
    LPA(float heuristic_weight, const Map &map);

    SearchResult startSearch(const EnvironmentOptions &options);

    // Functions which used by robot to explore new obstacles around him
    void pointNewObstacle(unsigned i, unsigned j, const EnvironmentOptions &options);

protected:
    struct open_node {
        Node node;
        bool is_closed;
        bool is_open;
    };

    SearchResult sresult;
    std::list<Node> lppath, hppath;
    std::unordered_map<uint_least64_t, Node> close;
    std::vector<open_cluster_t> open;
    std::vector<uint_least64_t> cluster_minimums;
    int open_size;
    float hweight;
public:
    Map current_graph;
};


#endif //D_LITE_LPA_H
