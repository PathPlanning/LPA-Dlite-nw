#include "mission.h"
#include "LPA.h"
#include "xmllogger.h"
#include "gl_const.h"
#include "astar.h"

Mission::Mission() {
    logger = NULL;
    search = NULL;
    fileName = NULL;
}

Mission::Mission(const char *FileName) {
    fileName = FileName;
    logger = NULL;
    search = NULL;
}

Mission::~Mission() {
    if (logger != NULL) delete logger;
    if (search != NULL) delete search;
}

bool Mission::getMap() {
    return map.getMap(fileName);
}

bool Mission::getConfig() {
    return config.getConfig(fileName);
}

bool Mission::createLog() {
    if (logger != NULL) delete logger;
    logger = new XmlLogger();
    logger->loglevel = config.LogParams[CN_LP_LEVEL];
    return logger->getLog(fileName, config.LogParams);
}

void Mission::createEnvironmentOptions() {
    options = EnvironmentOptions(config.SearchParams[CN_SP_AS], config.SearchParams[CN_SP_AD],
                                 config.SearchParams[CN_SP_CC], config.SearchParams[CN_SP_MT]);
}

void Mission::createSearch() {
    search = new LPA(config.SearchParams[CN_SP_HW], map);
}

void printPath(const SearchResult &sr, const Map &map) {
    for (int i = 0; i < map.height; ++i) {
        for (int j = 0; j < map.width; ++j) {
            bool is_path = false;
            for (Node node : *sr.lppath) {
                if (node.i == i && node.j == j) {
                    is_path = true;
                    break;
                }
            }
            if (is_path) {
                std::cout << '*';
            } else if (map.CellIsObstacle(i, j)) {
                std::cout << 1;
            } else {
                std::cout << 0;
            }
            std::cout << ' ';
        }
        std::cout << '\n';
    }
    std::cout << '\n';
}

void Mission::startSearch() {
    sr = search->startSearch(options);

    for (int i = map.height >> 2; i < map.height - (map.height >> 2); ++i) {
        int j = map.width >> 1;
        search->pointNewObstacle(i, j, options);
    }

    for (int j = map.width >> 2; j < map.width - (map.width >> 2); ++j) {
        int i = map.height >> 1;
        search->pointNewObstacle(i, j, options);
    }

    sr = search->startSearch(options);

    ISearch::SearchResult sr_astar = Astar(1).startSearch(logger, search->current_graph, options);
    sr.astar_pathlength = sr_astar.pathlength;
}

void Mission::printSearchResultsToConsole() {
    std::cout << "Path ";
    if (!sr.pathfound)
        std::cout << "NOT ";
    std::cout << "found!" << std::endl;
    std::cout << "numberofsteps=" << sr.numberofsteps << std::endl;
    std::cout << "nodescreated=" << sr.nodescreated << std::endl;
    if (sr.pathfound) {
        std::cout << "pathlength=" << sr.pathlength << std::endl;
        std::cout << "pathlength_scaled=" << sr.pathlength * map.cellSize << std::endl;
    }
    std::cout << "astar length=" << sr.astar_pathlength << '\n';
    std::cout << "time=" << sr.time << std::endl;
}

void Mission::saveSearchResultsToLog() {
    logger->writeToLogSummary(sr.numberofsteps, sr.nodescreated, sr.pathlength, sr.time, sr.astar_pathlength);
    if (sr.pathfound) {
        logger->writeToLogPath(*sr.lppath);
        logger->writeToLogHPpath(*sr.hppath);
        logger->writeToLogMap(map, *sr.lppath);
    } else
        logger->writeToLogNotFound();
    logger->saveLog();
}
