
#include "headers/pathfinder.h"

Pathfinder::Pathfinder()
{
    collectionControlMutex = std::make_shared<std::mutex>();
}

Pathfinder::~Pathfinder()
{
    for(NodeCollection &collection : nodeCollections)
        collection.scheduleThreadToStop();

    threadRunning = false;
    while(!threadClosed)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

/* Starts the solver.
 * The mapStatusChecker thread is then started.
 * All NodeCollection threads are then started and wait to be activated.
 */
void Pathfinder::startSolver()
{
    /* Start the thread that monitors the status of the map. Not needed to run solver.
     *
    threadRunning = true;
    threadClosed = false;
    mapStatusThread = std::shared_ptr<std::thread>(new std::thread(&Pathfinder::mapStatusChecker,this));
    mapStatusThread->detach();
    //*/

    // Set inital node and start solving the map.
    map->at(currentHeading->first).at(currentHeading->second)->setNodeAsInit();
    for(NodeCollection &collection : nodeCollections)
        collection.start();
}

void Pathfinder::setInitialPosition(std::shared_ptr<std::pair<std::size_t, std::size_t> > position)
{
    pauseSolver();
    currentHeading = position;

    for(std::size_t i = 0; i < map->size(); i++)
        for(std::size_t j = 0; j < map->at(i).size(); j++)
            map->at(i).at(j)->resetNode();

    std::shared_ptr<Node> currentHeadingNode = map->at(currentHeading->first).at(currentHeading->second);

    currentHeadingNode->setNodeAsInit();

    currentHeadingNode->unlockNodeReady();

    timeMeasureBegin = std::chrono::steady_clock::now();
    mapIsStable = false;

    resumeSolver();
}

void Pathfinder::setCurrentHeading(std::shared_ptr<std::pair<std::size_t, std::size_t> > heading)
{
    pauseSolver();

    /*
    for(const auto &it : *map.get())
        std::for_each(it.begin(), it.end(), std::bind(&Node::setUpdated, std::placeholders::_1, &temp));
        */

    /*
    for(auto &it : *map.get())
        for(auto &it2 : it)
            it2->setUpdated(false);
    //*/

    for(std::size_t i = 0; i < map->size(); i++)
        for(std::size_t j = 0; j < map->at(i).size(); j++)
            map->at(i).at(j)->setUpdated(false);

    // Previous
    //if(currentHeading) {
        //std::shared_ptr<Node> previousHeadingNode = map->at(currentHeading->first).at(currentHeading->second);

        //previousHeadingNode->setUpdated(false);
        //previousHeadingNode->setStable(true);
    //}

    // Current
    currentHeading = heading;

    std::shared_ptr<Node> currentHeadingNode = map->at(currentHeading->first).at(currentHeading->second);

    /*

    currentHeadingNode->setStable(false);
    currentHeadingNode->setUpdated(true);
    currentHeadingNode->setSource(*currentHeading.get());
    currentHeadingNode->setPointerToSource(std::make_shared<Node>());
    currentHeadingNode->setCost(0);
    currentHeadingNode->unlockNodeReady();
    //*/
    currentHeadingNode->setNodeAsInit();
    //previousHeadingNode->setCost(2);
    //previousHeadingNode->addToNextCost2(2, currentHeadingNode);

    timeMeasureBegin = std::chrono::steady_clock::now();
    mapIsStable = false;

    resumeSolver();
}

void Pathfinder::updatePenaltyOfNode(std::size_t row, std::size_t col, double penalty)
{
    pauseSolver();
    std::shared_ptr<Node> penaltyNode = map->at(row).at(col);
    penaltyNode->setPenalty(penalty);

    timeMeasureBegin = std::chrono::steady_clock::now();
    mapIsStable = false;

    resumeSolver();
}

void Pathfinder::updatePenaltyOfNodeGroup(std::vector<std::pair<std::size_t, std::size_t> > positions, double penalty)
{
    pauseSolver();

    for(std::pair<std::size_t, std::size_t> &pos : positions) {
        std::shared_ptr<Node> penaltyNode = map->at(pos.first).at(pos.second);
        penaltyNode->setPenalty(penalty);
    }

    timeMeasureBegin = std::chrono::steady_clock::now();
    mapIsStable = false;

    resumeSolver();
}

long Pathfinder::getCurrentComputationTime()
{
   if(mapIsStable)
       return std::chrono::duration_cast<std::chrono::milliseconds>(timeMeasureEnd - timeMeasureBegin).count();
   std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
   return std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - timeMeasureBegin).count();
}


/* Checks the current status of the map.
 * When the map is stable, the thread running this function sleeps.
 * When unstable, the stability of the map is checked every 100 ms. When stable, it returns to the wait state.
 */
void Pathfinder::mapStatusChecker()
{
    while(threadRunning) {
        waitForMapUnstable();
        timeMeasureBegin = std::chrono::steady_clock::now(); // Start to measure computation time of solver.
        waitForMapStable();
        timeMeasureEnd = std::chrono::steady_clock::now();
    }
    threadClosed = true;
}


/* Waits for the map to be unstable.
 * The stability of the map is checked every x ms.
 * Note that this stops the entire computation of the path every x ms.
 */
void Pathfinder::waitForMapUnstable()
{
    std::chrono::steady_clock::time_point stableBeginTime = std::chrono::steady_clock::now();
    int i = 0;
    while(checkMapStable() && mapIsStable && threadRunning) {
        if(++i > 50) {
            std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
            std::cout << "Map has been stable for: " << std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - stableBeginTime).count() << std::endl;
            i = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    mapIsStable = false;
}


/* Waits for the map to be stable.
 * The stability of the map is checked every x ms.
 * Note that this stops the entire computation of the path every x ms.
 */
void Pathfinder::waitForMapStable()
{
    int i = 0;
    while(!checkMapStable() && !mapIsStable && threadRunning) {
        if(++i > 50) {
            std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
            std::cout << "Stabilzing map. Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - timeMeasureBegin).count() << std::endl;
            i = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    mapIsStable = true;
}


/* Checks if the map is stable.
 * Stops all computation of the map while checking.
 */
bool Pathfinder::checkMapStable()
{
    pauseSolver();

    bool stable = true;
    for(std::size_t i = 0; i < map->size(); i++) {
        for(std::size_t j = 0; j < map->at(i).size(); j++) {
            if(!map->at(i).at(j)->getStable()) {
                stable = false;
                break;
            }
            if(!stable)
                break;
        }
    }

    resumeSolver();

    return stable;
}

void Pathfinder::pauseSolver()
{
    collectionControlMutex->lock();

    for(NodeCollection &collection : nodeCollections)
        collection.pause();

    for(NodeCollection &collection : nodeCollections)
        while(!collection.getIsPaused())
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void Pathfinder::resumeSolver()
{
    for(NodeCollection &collection : nodeCollections)
        collection.resume();

    collectionControlMutex->unlock();
}
