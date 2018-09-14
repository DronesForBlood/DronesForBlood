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
    bool getIsPaused() {std::cout << "CMON" << isPaused << std::endl; return isPaused;}


private:
    [[ noreturn ]] void nodeChecker();

private:
   std::vector<std::shared_ptr<Node>> nodes;
   std::shared_ptr<std::mutex> nodeReadyMutex;
   std::shared_ptr<std::thread> checkerThread;
   bool *checkNodesAgain = new bool(false);
   bool stable = false;
   bool pauseThread = false;
   bool isPaused = false;

};

#endif // NODECOLLECTION_H
