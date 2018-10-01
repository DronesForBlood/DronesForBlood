
#include "headers/pathshortener.h"

PathShortener::PathShortener()
{

}

PathShortener::~PathShortener()
{

}

void PathShortener::shortenPath(std::vector<std::pair<double, double> > path, std::vector<std::pair<double, double> > &shortPath, double epsilon)
{
    /*
    std::pair<double,double> p1(55.362397, 10.422111);
    std::pair<double,double> p2(55.359154, 10.427775);
    std::pair<double,double> p3(55.362420, 10.427049);
    double distance = perpendicularDistance(p1, p2, p3);
    std::cout << "Dist: " << distance << std::endl;
    //*/

    douglasPeucker(path, epsilon, shortPath);

    std::cout << path.size() << std::endl;
    std::cout << shortPath.size() << std::endl;
    //for(auto it : shortPath)
    //    std::cout << it.first << " " << it.second << std::endl;

}

void PathShortener::douglasPeucker(std::vector<std::pair<double, double> > &pointList, double epsilon, std::vector<std::pair<double,double>> &out)
{
    // Find the point with the maximum distance from line between start and end
    double dmax = 0.0;
    size_t index = 0;
    size_t end = pointList.size()-1;
    for(size_t i = 1; i < end; i++)
    {
        double d = perpendicularDistance(pointList[0], pointList[end], pointList[i]);
        if (d > dmax)
        {
            index = i;
            dmax = d;
        }
    }

    // If max distance is greater than epsilon, recursively simplify
    if(dmax > epsilon)
    {
        // Recursive call
        std::vector<std::pair<double,double>> recResults1;
        std::vector<std::pair<double,double>> recResults2;
        std::vector<std::pair<double,double>> firstLine(pointList.begin(), pointList.begin()+index+1);
        std::vector<std::pair<double,double>> lastLine(pointList.begin()+index, pointList.end());
        douglasPeucker(firstLine, epsilon, recResults1);
        douglasPeucker(lastLine, epsilon, recResults2);

        // Build the result list
        out.assign(recResults1.begin(), recResults1.end()-1);
        out.insert(out.end(), recResults2.begin(), recResults2.end());
    }
    else
    {
        //Just return start and end points
        out.clear();
        out.push_back(pointList[0]);
        out.push_back(pointList[end]);
    }
}

// If the coordinates are not in the same UTM zone, this will return 0. Should be changed so the the zone is locked.
double PathShortener::perpendicularDistance(std::pair<double, double> linePoint1, std::pair<double, double> linePoint2, std::pair<double, double> point)
{
    double UTMNorthing;
    double UTMEasting;

    std::string UTMZone1;
    RobotLocalization::NavsatConversions::LLtoUTM(linePoint1.first, linePoint1.second, UTMNorthing, UTMEasting, UTMZone1);
    linePoint1.first = UTMNorthing;
    linePoint1.second = UTMEasting;

    std::string UTMZone2;
    RobotLocalization::NavsatConversions::LLtoUTM(linePoint2.first, linePoint2.second, UTMNorthing, UTMEasting, UTMZone2);
    linePoint2.first = UTMNorthing;
    linePoint2.second = UTMEasting;

    std::string UTMZone3;
    RobotLocalization::NavsatConversions::LLtoUTM(point.first, point.second, UTMNorthing, UTMEasting, UTMZone3);
    point.first = UTMNorthing;
    point.second = UTMEasting;

    if(UTMZone1 != UTMZone2 || UTMZone1 != UTMZone3)
        return 0;

    double nominator = fabs((linePoint2.second - linePoint1.second) * point.first - (linePoint2.first - linePoint1.first) * point.second + linePoint2.first * linePoint1.second - linePoint2.second * linePoint1.first);
    double denominator = sqrt(pow(linePoint2.second - linePoint1.second, 2) + pow(linePoint2.first - linePoint1.first, 2));
    double distance = nominator/denominator;
    //std::cout << "Dist: " << distance << std::endl;
    return distance;
}
