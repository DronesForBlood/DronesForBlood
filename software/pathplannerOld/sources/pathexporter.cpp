/****************************************************************************
# exportKML
# Copyright (c) 2014-2018, Kjeld Jensen <kj@kjen.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/

/*
 * 2015-03-22 Kjeld Removed unnecessary trkptend() function
 * 2015-11-18 Kjeld Added optional absolute altitude mode
 * 2018-03-13 Kjeld Added a
*/

#include "headers/pathexporter.h"


PathExporter::PathExporter()
{

}

PathExporter::~PathExporter()
{

}

void PathExporter::exportToKml(std::vector<std::pair<double, double> > coordinates, std::string filename, std::string pathname, std::string description)
{
    begin(filename, pathname, description, 0.1);
    // color: use 'red' or 'green' or 'blue' or 'cyan' or 'yellow' or 'grey'
    // altitude: use 'absolute' or 'relativeToGround'
    trksegbegin("", "", "red", "absolute");
    for(auto coord : coordinates)
        trkpt(coord.first, coord.second, 10);
    trksegend();
    end();
}

void PathExporter::begin(std::string filename, std::string name, std::string description, double width)
{
    stream << std::setprecision(10);
    stream.open(filename + ".kml");
    stream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    stream << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
    stream << "<Document>\n";
    stream << "<name>" << name << "</name>\n";
    stream << "<description>" << description << "</description>\n";
    stream << "<Style id=\"red\">\n";
    stream << "  <LineStyle>\n";
    stream << "    <color>ff0000ff</color>\n";
    stream << "    <width>" << width << "</width>\n";
    stream << "  </LineStyle>\n";
    stream << "</Style>\n";
    stream << "<Style id=\"green\">\n";
    stream << "  <LineStyle>\n";
    stream << "    <color>ff00ff00</color>\n";
    stream << "    <width>" << width << "</width>\n";
    stream << "  </LineStyle>\n";
    stream << "</Style>\n";
    stream << "<Style id=\"blue\">\n";
    stream << "  <LineStyle>\n";
    stream << "    <color>ffff0000</color>\n";
    stream << "    <width>" << width << "</width>\n";
    stream << "  </LineStyle>\n";
    stream << "</Style>\n";
    stream << "<Style id=\"cyan\">\n";
    stream << "  <LineStyle>\n";
    stream << "    <color>ffffff00</color>\n";
    stream << "    <width>" << width << "</width>\n";
    stream << "  </LineStyle>\n";
    stream << "</Style>\n";
    stream << "<Style id=\"yellow\">\n";
    stream << "  <LineStyle>\n";
    stream << "    <color>ff00ffff</color>\n";
    stream << "    <width>" << width << "</width>\n";
    stream << "  </LineStyle>\n";
    stream << "</Style>\n";
    stream << "<Style id=\"grey\">\n";
    stream << "  <LineStyle>\n";
    stream << "    <color>ff888888</color>\n";
    stream << "    <width>" << width << "</width>\n";
    stream << "  </LineStyle>\n";
    stream << "</Style>\n";
}

void PathExporter::trksegbegin(std::string segname, std::string segdesc, std::string color, std::string altitude)
{
    stream << "<Placemark>\n";
    stream << "<name>" << segname << "</name>\n";
    stream << "<description>" << segdesc << "</description>\n";
    stream << "<styleUrl>#" << color << "</styleUrl>\n";
    stream << "<LineString>\n";
    if(altitude == "absolute")
        stream << "<altitudeMode>absolute</altitudeMode>\n";
    else if(altitude == "relativeToGround")
        stream << "<altitudeMode>relativeToGround</altitudeMode>\n";
    stream << "<coordinates>\n";
}

void PathExporter::trksegend()
{
    stream << "</coordinates>\n";
    stream << "</LineString>\n";
    stream << "</Placemark>\n";
}

void PathExporter::trkpt(double lat, double lon, double ele)
{
    stream << lon << "," << lat << "," << ele << "\n";
}

void PathExporter::end()
{
    stream << "</Document>\n";
    stream << "</kml>";
    stream.close();
}
