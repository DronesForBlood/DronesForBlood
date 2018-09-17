
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
    //currentHeading = currentPosition;
    // Start the thread that monitors the status of the map.
    //threadRunning = true;
    //threadClosed = false;
    //mapStatusThread = std::shared_ptr<std::thread>(new std::thread(&Pathfinder::mapStatusChecker,this));
    //mapStatusThread->detach();

    // Set inital node and start solving the map.
    map->at(currentPosition->first).at(currentPosition->second)->setNodeAsInit();
    for(NodeCollection &collection : nodeCollections)
        collection.start();
}

void Pathfinder::setCurrentHeading(std::shared_ptr<std::pair<std::size_t, std::size_t> > heading)
{
    pauseSolver();

    // Previous
    if(currentHeading)
        map->at(currentHeading->first).at(currentHeading->second)->setUpdated(false);

    /*
    for(std::size_t i = 0; i < map->size(); i++)
        for(std::size_t j = 0; j < map->at(i).size(); j++)
            map->at(i).at(j)->setUpdated(false);
            */


    // Current
    currentHeading = heading;
    map->at(currentHeading->first).at(currentHeading->second)->setStable(false);
    map->at(currentHeading->first).at(currentHeading->second)->setUpdated(true);
    map->at(currentHeading->first).at(currentHeading->second)->setSource(*currentHeading.get());
    map->at(currentHeading->first).at(currentHeading->second)->setPointerToSource(std::make_shared<Node>());
    map->at(currentHeading->first).at(currentHeading->second)->setCost(0);

    map->at(currentHeading->first).at(currentHeading->second)->unlockNodeReady();

    timeMeasureBegin = std::chrono::steady_clock::now();
    mapIsStable = false;

    resumeSolver();
}

void Pathfinder::updatePenaltyOfNode(std::size_t row, std::size_t col, double penalty)
{
    pauseSolver();

    for(std::size_t i = 0; i < map->size(); i++)
        for(std::size_t j = 0; j < map->at(i).size(); j++)
            map->at(i).at(j)->setUpdated(false);

    map->at(row).at(col)->setPenalty(penalty);

    map->at(currentHeading->first).at(currentHeading->second)->setStable(false);
    map->at(currentHeading->first).at(currentHeading->second)->setUpdated(true);
    map->at(currentHeading->first).at(currentHeading->second)->setSource(*currentHeading.get());
    map->at(currentHeading->first).at(currentHeading->second)->setPointerToSource(std::make_shared<Node>());

    map->at(currentHeading->first).at(currentHeading->second)->unlockNodeReady();

    timeMeasureBegin = std::chrono::steady_clock::now();
    mapIsStable = false;

    resumeSolver();

    std::cout << "Penalty of node updated!" << std::endl;
}

void Pathfinder::updatePenaltyOfNodeGroup(std::vector<std::pair<std::size_t, std::size_t> > positions, double penalty)
{

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

    for(NodeCollection &collection : nodeCollections) {
        while(!collection.getIsPaused()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void Pathfinder::resumeSolver()
{
    for(NodeCollection &collection : nodeCollections)
        collection.resume();

    collectionControlMutex->unlock();
}
