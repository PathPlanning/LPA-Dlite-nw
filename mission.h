#ifndef MISSION_H
#define	MISSION_H

#include "map.h"
#include "config.h"
#include "LPA.h"
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include "xmllogger.h"

class Mission
{
    public:
        Mission();
        Mission (const char* fileName);
        ~Mission();

        bool getMap();
        bool getConfig();
        bool createLog();
        void createSearch();
        void createEnvironmentOptions();
        void startSearch();
        void printSearchResultsToConsole();
        void saveSearchResultsToLog();

    private:
        Map                     map;
        Config                  config;
        EnvironmentOptions      options;
        LPA*                search;
        ILogger*                logger;
        const char*             fileName;
        SearchResult            sr;
};

#endif

