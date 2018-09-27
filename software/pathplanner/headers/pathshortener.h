#ifndef PATHSHORTENER_H
#define PATHSHORTENER_H

#include <utility>
#include <vector>
#include <math.h>
#include <iostream>
#include <iomanip>

#include "headers/defines.h"
#include "headers/coordconverter.h"

class PathShortener
{
public:
    PathShortener();
    ~PathShortener();

    void shortenPath(std::vector<std::pair<double, double>> path, std::vector<std::pair<double, double>> &shortPath, double epsilon);

private:
    void douglasPeucker(std::vector<std::pair<double, double> > &pointList, double epsilon, std::vector<std::pair<double, double> > &out);
    double perpendicularDistance(std::pair<double, double> linePoint1, std::pair<double, double> linePoint2, std::pair<double, double> point);

private:

};

#endif // PATHSHORTENER_H
