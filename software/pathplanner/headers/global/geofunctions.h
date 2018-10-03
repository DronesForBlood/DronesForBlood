#ifndef GEOFUNCTIONS_H
#define GEOFUNCTIONS_H

#include <utility>
#include <math.h>
#include <vector>

#include "headers/global/defines.h"

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

/*
 * https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
 *
 * Copyright (c) 1970-2003, Wm. Randolph Franklin
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimers.
 * 2. Redistributions in binary form must reproduce the above copyright notice in the documentation and/or other materials provided with the distribution.
 * 3. The name of W. Randolph Franklin may not be used to endorse or promote products derived from this Software without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
static inline bool pointIsInsidePolygon(std::vector<std::pair<double,double>> &polygonCoordinates, std::pair<double,double> point)
{
    std::size_t i, j;
    bool c = false;
    std::size_t nvert = polygonCoordinates.size();
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
      if ( ((polygonCoordinates[i].second>point.second) != (polygonCoordinates[j].second>point.second)) &&
       (point.first < (polygonCoordinates[j].first-polygonCoordinates[i].first) * (point.second-polygonCoordinates[i].second) / (polygonCoordinates[j].second-polygonCoordinates[i].second) + polygonCoordinates[i].first) )
         c = !c;
    }
    return c;
}

}

#endif // GEOFUNCTIONS_H
