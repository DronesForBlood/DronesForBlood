#ifndef GEOFUNCTIONS_H
#define GEOFUNCTIONS_H

#include <utility>
#include <math.h>
#include <vector>
#include <iostream>

#include "defines.hpp"

namespace GeoFunctions
{

static inline double calcMeterDistanceBetweensCoords(std::pair<double, double> startCoord, std::pair<double, double> endCoord)
{
    startCoord.first = startCoord.first * PI / 180.;
    endCoord.first = endCoord.first * PI / 180.;
    startCoord.second = startCoord.second * PI / 180.;
    endCoord.second = endCoord.second * PI / 180.;

    if(startCoord.first - endCoord.first < 0.0000001 && startCoord.second - endCoord.second < 0.0000001 && startCoord.first - endCoord.first > -0.0000001 && startCoord.second - endCoord.second > -0.0000001)
        return 0;

    double distance_radians = acos(sin(startCoord.first) * sin(endCoord.first) + cos(startCoord.first) * cos(endCoord.first) * cos(startCoord.second - endCoord.second));
    double distanceMeters = distance_radians * RADIUS_EARTH_METERS;
    return distanceMeters;
}

static inline double calcAngle(std::pair<double, double> startCoord, std::pair<double, double> endCoord)
{
    startCoord.first = startCoord.first * PI / 180.;
    endCoord.first = endCoord.first * PI / 180.;
    startCoord.second = startCoord.second * PI / 180.;
    endCoord.second = endCoord.second * PI / 180.;

    double dLon = (endCoord.second - startCoord.second);

    double y = sin(dLon) * cos(endCoord.first);
    double x = cos(startCoord.first) * sin(endCoord.first) - sin(startCoord.first) * cos(endCoord.first) * cos(dLon);

    double brng = atan2(y, x);

    brng = brng / PI * 180;
    brng = int(brng + 360) % 360;

    //brng = 360 - brng; // count degrees counter-clockwise - remove to make clockwise

    return brng;
}

}

#endif // GEOFUNCTIONS_H
