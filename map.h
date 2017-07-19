#ifndef MAP_H
#define	MAP_H
#include <iostream>
#include "gl_const.h"
#include <sstream>
#include <string>
#include <algorithm>
class Map
{
    public:
        Map();
        Map(unsigned width, unsigned height, std::pair<unsigned, unsigned> start, std::pair<unsigned, unsigned> goal);
        Map(const Map& orig);

        bool getMap(const char *FileName);
        bool CellIsTraversable (int i, int j) const;
        bool CellOnGrid (int i, int j) const;
        bool CellIsObstacle(int i, int j) const;

        unsigned     height, width;
        unsigned     start_i, start_j;
        unsigned     goal_i, goal_j;
        double  cellSize;
        std::vector<std::vector<int>>   Grid;
};

#endif

