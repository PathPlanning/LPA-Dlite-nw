#ifndef ISEARCH_H
#define ISEARCH_H

#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include "node.h"
#include <list>
#include <unordered_map>

class DLiteSearch {
public:
    DLiteSearch();

    DLiteSearch(float heuristic_weigh, unsigned map_width, unsigned map_height, int view_radius);

protected:
    typedef std::pair<double, double> keytype;
    typedef std::unordered_map<uint_least64_t, Node> open_claster_t;

    Node findMin() const;

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

    void updateVertex(Node &node, const Node &target_node, const EnvironmentOptions &options, uint_fast32_t map_width);

    void updateRHS(Node &node, const Map &map, const EnvironmentOptions &options) const;

    // recomputes shortest path using vertex in open set. Open must contain at least one vertex.
    void computePath(Node finish_node, const Map &map, const EnvironmentOptions &options);

    // Functions which used by robot to explore new obstacles around him
    int exploreGraph(const Node &position, const Map &real_map, const EnvironmentOptions &options);

    void pointNewObstacle(unsigned i, unsigned j, const EnvironmentOptions &options, const Node &start_node);

public:
    void setStartPosition(const Node &start);

    void changeGoal(Node new_goal);

    void makeStep(const EnvironmentOptions &options);

    SearchResult goToGoal(const Map &real_map, const EnvironmentOptions &options);

    std::list<Node> constructFoundPath() const;


protected:
    struct open_node {
        Node node;
        bool is_open = false;
    };
    SearchResult sresult;
    std::list<Node> lppath, hppath;
    std::unordered_map<uint_least64_t, Node> close;
    std::vector<std::vector<open_node>> open;
    std::vector<uint_least64_t> cluster_minimums;
    int open_size;
    uint_least64_t number_of_steps;
    float hweight;
    double key_modifier;
    Node prev;
    bool graph_has_changed;
public:
    Map explored_graph;
    Node position;
    Node goal;
    int view_radius;
};

#endif
