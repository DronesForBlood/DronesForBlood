#ifndef PATHEXPORTER_H
#define PATHEXPORTER_H

#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>

class PathExporter
{
public:
    PathExporter();
    ~PathExporter();
    void exportToKml(std::vector<std::pair<double,double>> coordinates, std::string filename, std::string pathname, std::string description);

private:
    void begin(std::string filename, std::string name, std::string description, double width);
    void trksegbegin(std::string segname, std::string segdesc, std::string color, std::string altitude);
    void trksegend();
    void trkpt(double lat, double lon, double ele);
    void end();

private:
    std::ofstream stream;

};

#endif // PATHEXPORTER_H
