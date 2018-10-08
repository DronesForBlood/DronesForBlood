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

class Visualizer
{
public:
    Visualizer();
    Visualizer(int mapRows, int mapCols);
    ~Visualizer();

    void setColorOfPixel(std::pair<int,int> pixel, int r, int g, int b);
    void printCurrentPositionImage(std::pair<int, int> newPosition);
    void printPathImage(std::vector<std::pair<size_t, size_t> > &path, std::pair<size_t, size_t> currentHeading);
    void printShortPathImage(std::vector<std::pair<size_t, size_t>> &shortPath, std::pair<size_t, size_t> currentHeading);

private:
    bool isInsideMap(int row, int col);

private:
   cv::Mat pathImage;


   int squareSize = 5;
    std::mutex mutex;
   std::pair<int,int> previousPosition;
   std::vector<std::pair<size_t, size_t>> currentPath;
   std::vector<std::pair<size_t, size_t>> currentShortPath;

};

#endif // VISUALIZER_H
