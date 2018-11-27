#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <utility>
#include <vector>
#include <memory>
#include <math.h>
#include <iostream>
#include <thread>
#include <mutex>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "node.h"

class Visualizer
{
public:
    Visualizer();
    Visualizer(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, int mapRows, int mapCols);
    ~Visualizer();

    void setNewMap(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, int mapRows, int mapCols);
    void setCurrentPosition(std::pair<int, int> newPosition);
    void setCurrentPath(std::vector<std::pair<size_t, size_t> > path);
    void setCurrentShortPath(std::vector<std::pair<size_t, size_t>> shortPath);
    void setCurrentHeading(std::pair<size_t, size_t> heading);
    void printImage();

private:
    void printCurrentPositionImage();
    void printPathImage();
    void printShortPathImage();
    bool isInsideMap(int row, int col);


private:
   cv::Mat pathImage;
   std::shared_ptr<std::thread> printThread;
   std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map;

   bool threadRunning = false;
   bool threadClosed = true;

   int squareSize = 5;
   std::mutex editMutex;
   std::pair<int,int> previousPosition;
   std::pair<int,int> currentPosition;
   std::vector<std::pair<size_t, size_t>> previousPath;
   std::vector<std::pair<size_t, size_t>> currentPath;
   std::vector<std::pair<size_t, size_t>> previousShortPath;
   std::vector<std::pair<size_t, size_t>> currentShortPath;
   std::pair<size_t, size_t> currentHeading;



};

#endif // VISUALIZER_H
