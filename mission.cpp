#include "mission.h"
#include "dsearch.h"
#include "xmllogger.h"
#include "gl_const.h"

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
    search = new DLiteSearch(config.SearchParams[CN_SP_HW], map.width, map.height, 1);
}

void Mission::startSearch() {
    search->setStartPosition({map.start_i, map.start_j});
    search->changeGoal({map.goal_i, map.goal_j});
    sr = search->goToGoal(map, options);
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
    std::cout << "time=" << sr.time << std::endl;
}

void Mission::saveSearchResultsToLog() {
    logger->writeToLogSummary(sr.numberofsteps, sr.nodescreated, sr.pathlength, sr.time, map.cellSize);
    if (sr.pathfound) {
        logger->writeToLogPath(*sr.lppath);
        logger->writeToLogHPpath(*sr.hppath);
        logger->writeToLogMap(map, *sr.lppath);
    } else
        logger->writeToLogNotFound();
    logger->saveLog();
}
