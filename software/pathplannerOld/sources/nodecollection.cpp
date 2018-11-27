
#include "headers/nodecollection.h"



NodeCollection::NodeCollection()
{
    nodeReadyMutex = std::shared_ptr<std::mutex>(new std::mutex);
    pauseMutex = std::shared_ptr<std::mutex>(new std::mutex);
}

NodeCollection::~NodeCollection()
{
    threadRunning = false;
    while(!threadClosed)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
    pauseMutex->lock();
    pauseThread = true;
}

void NodeCollection::resume()
{
    pauseThread = false;
    pauseMutex->unlock();
}

void NodeCollection::scheduleThreadToStop()
{
    threadRunning = false;
}

void NodeCollection::nodeChecker()
{
    while(threadRunning) {
        if(pauseThread && threadRunning) {
            isPaused = true;
            pauseMutex->lock();
            pauseMutex->unlock();
            isPaused = false;
        }

        while(!*checkNodesAgain && threadRunning && !pauseThread)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        //if(pauseThread)
        //    continue;

        nodeReadyMutex->lock();
        *checkNodesAgain = false;
        nodeReadyMutex->unlock();

        for(std::shared_ptr<Node> &node : nodes)
            if(!node->getStable() && node->getUpdated())
                node->checkAndUpdateNeighbors();
    }

    threadClosed = true;
}
