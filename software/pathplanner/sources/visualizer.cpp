
#include "headers/visualizer.h"

Visualizer::Visualizer()
{

}

Visualizer::Visualizer(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, int mapRows, int mapCols)
{
    map = aMap;
    pathImage = cv::Mat(mapRows, mapCols, CV_8UC3, cv::Scalar(0, 0, 0));

    previousPosition.first = -1;
    previousPosition.second = -1;

    cv::imshow("DroneSimulator 2000", pathImage);
    cv::waitKey(100);

    threadRunning = true;
    threadClosed = false;
    printThread = std::shared_ptr<std::thread>(new std::thread(&Visualizer::printImage,this));
    printThread->detach();
}

Visualizer::~Visualizer()
{
    threadRunning = false;
    while(!threadClosed)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void Visualizer::setCurrentPosition(std::pair<int, int> newPosition)
{
       previousPosition = currentPosition;
       currentPosition = newPosition;
}

void Visualizer::setCurrentPath(std::vector<std::pair<size_t, size_t> > path)
{
    previousPath = currentPath;
    currentPath = path;
}

void Visualizer::setCurrentShortPath(std::vector<std::pair<size_t, size_t> > shortPath)
{
    previousShortPath = currentShortPath;
    currentShortPath = shortPath;
}

void Visualizer::setCurrentHeading(std::pair<size_t, size_t> heading)
{
    currentHeading = heading;
}

void Visualizer::printImage()
{
    while(threadRunning) {
        pathImage = cv::Scalar(0,0,0);

        for(auto &it : *map.get())
            for(auto &it2 : it) {
                std::pair<size_t, size_t> index = it2->getNodeIndex();
                cv::Vec3b *pixelColor = &pathImage.at<cv::Vec3b>(cv::Point(int(index.second), int(index.first)));
                cv::Scalar nodeColor = it2->getColor();
                for(int i = 0; i < 3; i++) {
                    if(nodeColor[i] > 255)
                        pixelColor->val[i] = 255;
                    else if(nodeColor[i] < 0)
                        pixelColor->val[i] = 0;
                    else
                        pixelColor->val[i] = uchar(nodeColor[i]);
                }
            }


        printCurrentPositionImage();
        printPathImage();
        printShortPathImage();

        cv::imshow("DroneSimulator 2000", pathImage);
        cv::waitKey(1);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    threadClosed = true;
}

void Visualizer::printCurrentPositionImage()
{
    if(previousPosition.first != -1)
        for(int j = int(previousPosition.second) - squareSize; j < int(previousPosition.second) + squareSize; j++)
            for(int k = int(previousPosition.first) - squareSize; k < int(previousPosition.first) + squareSize; k++) {
                if(isInsideMap(k,j)) {
                    cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(j), int(k)));
                    color->val[1] = 0;
                }
            }

    for(int j = int(currentPosition.second) - squareSize; j < int(currentPosition.second) + squareSize; j++)
        for(int k = int(currentPosition.first) - squareSize; k < int(currentPosition.first) + squareSize; k++) {
            if(isInsideMap(k,j)) {
                cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(j), int(k)));
                color->val[1] = 255;
            }
        }
}

void Visualizer::printPathImage()
{
    for(std::size_t i = 0; i < previousPath.size(); i++) {
        cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(previousPath[i].second), int(previousPath[i].first)));
        color->val[1] = 0;
    }

    for(std::size_t i = 0; i < currentPath.size(); i++) {
        cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(currentPath[i].second), int(currentPath[i].first)));
        color->val[1] = 200;
    }
}

void Visualizer::printShortPathImage()
{
    for(std::size_t i = 0; i < previousShortPath.size(); i++) {
        for(int j = int(previousShortPath[i].second) - squareSize; j < int(previousShortPath[i].second) + squareSize; j++)
            for(int k = int(previousShortPath[i].first) - squareSize; k < int(previousShortPath[i].first) + squareSize; k++) {
                if(isInsideMap(k,j)) {
                    cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(j), int(k)));
                    color->val[1] = 0;
                }
            }
    }

    for(std::size_t i = 0; i < currentShortPath.size(); i++) {
        for(int j = int(currentShortPath[i].second) - squareSize; j < int(currentShortPath[i].second) + squareSize; j++)
            for(int k = int(currentShortPath[i].first) - squareSize; k < int(currentShortPath[i].first) + squareSize; k++) {
                if(isInsideMap(k,j)) {
                    cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(j), int(k)));
                    color->val[1] = 200;
                }

            }
    }
}

bool Visualizer::isInsideMap(int row, int col)
{
    if(row < 0 || col < 0)
        return false;

    if(row >= pathImage.rows)
        return false;

    if(col >= pathImage.cols)
        return false;

    return true;
}
