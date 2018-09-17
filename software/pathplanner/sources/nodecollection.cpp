
#include "headers/nodecollection.h"



NodeCollection::NodeCollection()
{
    nodeReadyMutex = std::shared_ptr<std::mutex>(new std::mutex);
}

NodeCollection::~NodeCollection()
{
    threadRunning = false;
    while(!threadClosed)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));


    //std::cout << "CLOSED 2!" << std::endl;

}

void NodeCollection::addNode(std::shared_ptr<Node> node)
{
    node->setNodeReadyMutex(nodeReadyMutex);
    node->setCheckNodesAgain(checkNodesAgain);
    nodes.push_back(node);
}

void NodeCollection::start()
{
    threadRunning = true;
    threadClosed = false;
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

void NodeCollection::scheduleThreadToStop()
{
    threadRunning = false;
}

void NodeCollection::nodeChecker()
{
    while(threadRunning) {

        if(pauseThread)
            isPaused = true;
        while(pauseThread && threadRunning)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

        isPaused = false;

        while(!*checkNodesAgain && threadRunning)
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
    //std::cout << "CLOSED!" << std::endl;
    threadClosed = true;
}
