
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

    resumeSolver();
}

void Pathfinder::setCurrentHeading(std::shared_ptr<std::pair<std::size_t, std::size_t> > heading)
{
    pauseSolver();

    for(auto &it : *map.get())
        for(auto &it2 : it)
            it2->setUpdated(false);

    currentHeading = heading;

    map->at(currentHeading->first).at(currentHeading->second)->setNodeAsInit();

    resumeSolver();
}

void Pathfinder::updatePenaltyOfNode(std::size_t row, std::size_t col, double penalty)
{
    pauseSolver();

    map->at(row).at(col)->setPenalty(penalty);

    resumeSolver();
}

void Pathfinder::updatePenaltyOfNodeGroup(std::vector<std::shared_ptr<Node>> &nodes, double penalty)
{
    pauseSolver();

    /*
    for(auto it : nodes)
        it->setUpdated(false);

    for(std::size_t i = 0; i < map->size(); i++)
        for(std::size_t j = 0; j < map->at(i).size(); j++)
            map->at(i).at(j)->setStable(false);
            */

    //*
    size_t nrBunches = std::size_t(nodes.size()/50.);
    if(nrBunches > 8)
        nrBunches = 8;
    if(nrBunches < 1)
        nrBunches = 1;

    size_t bunch_size = std::size_t(nodes.size()/nrBunches);
    std::vector<std::vector<std::shared_ptr<Node>>> bunches((nodes.size() + bunch_size) / bunch_size);

    for(size_t i = 0; i < nodes.size(); i += bunch_size) {
        auto last = std::min(nodes.size(), i + bunch_size);
        auto index = i / bunch_size;
        auto& vec = bunches[index];
        vec.reserve(last - i);
        move(nodes.begin() + i, nodes.begin() + last, back_inserter(vec));
    }

    if(bunches.back().size() < 25) {
        bunches[bunches.size()-2].insert(bunches[bunches.size()-2].end(), bunches.back().begin(), bunches.back().end());
        bunches.pop_back();
    }

    std::vector<bool*> doneVector;
    for(size_t i = 0; i < nrBunches; i++) {
        doneVector.push_back(new bool(false));
        std::shared_ptr<std::thread> thread = std::shared_ptr<std::thread>(new std::thread(&Pathfinder::threadTest, this, bunches[i], penalty, doneVector.back()));
        thread->detach();
    }

    bool repeat = true;
    while(repeat) {
        repeat = false;
        for(bool *done : doneVector)
            if(!*done)
                repeat = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for(bool *done : doneVector)
        delete done;
    //*/

    //for(auto &node : nodes)
    //    node->setPenalty(penalty);

    resumeSolver();
}

void Pathfinder::threadTest(std::vector<std::shared_ptr<Node> > nodes, double penalty, bool *done)
{
    for(auto &node : nodes)
        node->setPenalty(penalty);
    *done = true;
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
