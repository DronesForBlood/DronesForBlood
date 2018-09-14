
#include "headers/nodecollection.h"



NodeCollection::NodeCollection()
{
    nodeReadyMutex = std::shared_ptr<std::mutex>(new std::mutex);
}

NodeCollection::~NodeCollection()
{

}

void NodeCollection::addNode(std::shared_ptr<Node> node)
{
    node->setNodeReadyMutex(nodeReadyMutex);
    node->setCheckNodesAgain(checkNodesAgain);
    nodes.push_back(node);
}

void NodeCollection::start()
{
    checkerThread = std::shared_ptr<std::thread>(new std::thread(&NodeCollection::nodeChecker,this));
    checkerThread->detach();
}

void NodeCollection::pause()
{
    pauseThread = true;
    nodeReadyMutex->lock();
    *checkNodesAgain = true;
    nodeReadyMutex->unlock();
}

void NodeCollection::resume()
{
    pauseThread = false;
}

void NodeCollection::nodeChecker()
{
    while(true) {

        if(pauseThread)
            isPaused = true;
        while(pauseThread)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

        isPaused = false;

        while(!*checkNodesAgain)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

        nodeReadyMutex->lock();
        *checkNodesAgain = false;
        nodeReadyMutex->unlock();

        for(std::shared_ptr<Node> &node : nodes) {
            if(!node->getStable()) {
                node->checkAndUpdateNeighbors();
            }
        }
    }
}
