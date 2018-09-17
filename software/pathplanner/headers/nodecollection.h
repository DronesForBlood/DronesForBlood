#ifndef NODECOLLECTION_H
#define NODECOLLECTION_H

#include <utility>
#include <vector>
#include <memory>
#include <math.h>
#include <iostream>
#include <thread>
#include <mutex>

#include "node.h"

class NodeCollection
{
public:
    NodeCollection();
    ~NodeCollection();

    void addNode(std::shared_ptr<Node> node);
    void start();
    void pause();
    void resume();
    void scheduleThreadToStop();
    bool getIsPaused() {return isPaused;}


private:
    void nodeChecker();

private:
   std::vector<std::shared_ptr<Node>> nodes;
   std::shared_ptr<std::mutex> nodeReadyMutex;
   std::shared_ptr<std::thread> checkerThread;
   bool *checkNodesAgain = new bool(false);
   bool stable = false;
   bool pauseThread = false;
   bool isPaused = true;
   bool threadRunning = false;
   bool threadClosed = true;

};

#endif // NODECOLLECTION_H
