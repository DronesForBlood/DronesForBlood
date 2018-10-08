
#include "headers/visualizer.h"

Visualizer::Visualizer()
{

}

Visualizer::Visualizer(int mapRows, int mapCols)
{
    pathImage = cv::Mat(mapRows, mapCols, CV_8UC3, cv::Scalar(0, 0, 0));

    previousPosition.first = -1;
    previousPosition.second = -1;

    cv::imshow("DroneSimulator 2000", pathImage);
    cv::waitKey(100);
}

Visualizer::~Visualizer()
{

}

void Visualizer::setColorOfPixel(std::pair<int, int> pixel, int r, int g, int b)
{
    cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(pixel.second, pixel.first));
    if(b >= 0)
        color->val[0] = uchar(b);
    if(g >= 0)
        color->val[1] = uchar(g);
    if(r >= 0)
        color->val[2] = uchar(r);

}

void Visualizer::printCurrentPositionImage(std::pair<int, int> newPosition)
{
    if(previousPosition.first != -1)
        for(int j = int(previousPosition.second) - squareSize; j < int(previousPosition.second) + squareSize; j++)
            for(int k = int(previousPosition.first) - squareSize; k < int(previousPosition.first) + squareSize; k++) {
                if(isInsideMap(k,j)) {
                    cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(j), int(k)));
                    color->val[0] = 0;
                }
            }

    for(int j = int(newPosition.second) - squareSize; j < int(newPosition.second) + squareSize; j++)
        for(int k = int(newPosition.first) - squareSize; k < int(newPosition.first) + squareSize; k++) {
            if(isInsideMap(k,j)) {
                cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(j), int(k)));
                color->val[0] = 255;
            }
        }

    previousPosition = newPosition;

    cv::imshow("DroneSimulator 2000", pathImage);
    cv::waitKey(1);
}

void Visualizer::printPathImage(std::vector<std::pair<size_t, size_t> > &path, std::pair<size_t, size_t> currentHeading)
{
    std::size_t currentPositionIndex = 0;
    for(std::size_t i = 0; i < currentPath.size(); i++) {
        if(currentPath[i] == currentHeading) {
            currentPositionIndex = i;
            break;
        }
        cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(currentPath[i].second), int(currentPath[i].first)));
        color->val[1] = 0;
    }

    if(currentPositionIndex != 0)
        for(std::size_t i = currentPositionIndex; i < currentPath.size(); i++) {
            cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(currentPath[i].second), int(currentPath[i].first)));
            color->val[1] = 100;
        }

    for(std::size_t i = 0; i < path.size(); i++) {
        cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(path[i].second), int(path[i].first)));
        color->val[1] = 255;
    }

    currentPath = path;

    cv::imshow("DroneSimulator 2000", pathImage);
    cv::waitKey(1);
}

void Visualizer::printShortPathImage(std::vector<std::pair<size_t, size_t> > &shortPath, std::pair<size_t, size_t> currentHeading)
{
    std::size_t currentPositionIndex = 0;
    for(std::size_t i = 0; i < currentShortPath.size(); i++) {
        if(currentShortPath[i] == currentHeading) {
            currentPositionIndex = i;
            break;
        }
        for(int j = int(currentShortPath[i].second) - squareSize; j < int(currentShortPath[i].second) + squareSize; j++)
            for(int k = int(currentShortPath[i].first) - squareSize; k < int(currentShortPath[i].first) + squareSize; k++) {
                if(isInsideMap(k,j)) {
                    cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(j), int(k)));
                    color->val[1] = 0;
                }
            }
    }

    if(currentPositionIndex != 0)
        for(std::size_t i = currentPositionIndex; i < currentShortPath.size(); i++) {
            for(int j = int(currentShortPath[i].second) - squareSize; j < int(currentShortPath[i].second) + squareSize; j++)
                for(int k = int(currentShortPath[i].first) - squareSize; k < int(currentShortPath[i].first) + squareSize; k++) {
                    if(isInsideMap(k,j)) {
                        cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(j), int(k)));
                        color->val[1] = 100;
                    }
                }
        }

    for(std::size_t i = 0; i < shortPath.size(); i++) {
        for(int j = int(shortPath[i].second) - squareSize; j < int(shortPath[i].second) + squareSize; j++)
            for(int k = int(shortPath[i].first) - squareSize; k < int(shortPath[i].first) + squareSize; k++) {
                if(isInsideMap(k,j)) {
                    cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(j), int(k)));
                    color->val[1] = 255;
                }

            }
    }

    currentShortPath = shortPath;

    cv::imshow("DroneSimulator 2000", pathImage);
    cv::waitKey(1);
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
