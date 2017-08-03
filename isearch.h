#ifndef ISEARCH_H
#define ISEARCH_H
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include <unordered_map>

class ISearch
{
public:
    ISearch();
    virtual ~ISearch(void);

    struct SearchResult
    {
        bool pathfound;
        float pathlength; //if path not found, then pathlength=0
        const std::list<AstarNode> *lppath;
        const std::list<AstarNode> *hppath;
        unsigned int nodescreated; //|OPEN| + |CLOSE| = total number of nodes saved in memory during search process.
        unsigned int numberofsteps; //number of iterations made by algorithm to find a solution
        double time;
        SearchResult()
        {
            pathfound = false;
            pathlength = 0;
            lppath = nullptr;
            hppath = nullptr;
            nodescreated = 0;
            numberofsteps = 0;
            time = 0;
        }
    };

    ISearch::SearchResult startSearch(ILogger *Logger, const Map &Map, const EnvironmentOptions &options);

protected:
    AstarNode findMin();
    virtual void addOpen(AstarNode newNode);
    virtual double computeHFromCellToCell(int start_i, int start_j, int fin_i, int fin_j, const EnvironmentOptions &options) = 0;
    virtual std::list<AstarNode> findSuccessors(AstarNode curNode, const Map &map, const EnvironmentOptions &options);
    virtual void makePrimaryPath(AstarNode curNode);//Makes path using back pointers
    virtual void makeSecondaryPath();//Makes another type of path(sections or points)
    virtual AstarNode resetParent(AstarNode current, AstarNode parent, const Map &map, const EnvironmentOptions &options) {return current;}//Function for Theta*
    virtual bool stopCriterion();

    ISearch::SearchResult                    sresult;
    std::list<AstarNode>                 lppath, hppath;
    std::unordered_map<int,AstarNode>    close;
    std::vector<std::list<AstarNode>>    open;
    int                             openSize;
    double                          hweight;//weight of h-value
    bool                            breakingties;//flag that sets the priority of nodes in addOpen function when their F-values is equal

};
#endif
