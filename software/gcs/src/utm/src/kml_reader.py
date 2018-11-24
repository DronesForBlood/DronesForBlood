#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Description: KML parser for the no-fly zones obtained from https://www.droneluftrum.dk/
             The KML parser can handle Document.Placemark.Polygon, Document.Placemark.MultiGeometry.Polygon (generates sub placemarks with a modifies ID), and Document.Folder.Placemark.Polygon (Kjelds generated no-fly zone data format)
             The parsing can be verfied by comparing the results to the results on https://www.techgen.dk/msc/kml-viewer.html
    License: BSD 3-Clause
    Author: Tobias Lundby (tolu@mmmi.sdu.dk)
"""

from pykml import parser # KML file parser
#from get_no_fly_zones import get_no_fly_zones

import time

class kml_no_fly_zones_parser():
    def __init__(self, debug = False):
        """
        Init method
        Input: optional debug parameter which toogles debug messages
        Output: none
        """
        self.debug = debug
        self.placemarks = 0

    """def download_and_parse_data(self):"""
    """
        Downloads the data using the get_no_fly_zones class and then parses it usign the general method
        Input: none
        Output: bool from parsing
    """
    """
        downloader_module = get_no_fly_zones(self.debug)
        downloader_module.download_zones()
        file = parser.fromstring(downloader_module.get_data())
        return self.parse_data(file)"""

    def parse_file(self, file_name_in):
        """
        Parses the input file given by the already obtained filename; support reload
        Input: filename along
        Output: bool from parsing
        """
        try:
            with open(file_name_in) as f:
                file = parser.parse(f).getroot()
                return self.parse_data(file)
        except IOError:
            print ('KML file does not exist; skipping import')
            return False

    def parse_data(self, data):
        """
        Parses the input data
        Input: data
        Output: bool (False: parsing failed, True: parsing successful)
        """
        self.coordinate3d_combined = []
        itr = 0 # Incremented in the beginning so first element has itr=1
        try:
            data.Document.Placemark
        except AttributeError:
            if self.debug:
                print ("This is another sort of file, presumable the one provided by Kjeld")
            try:
                data.Document.Folder
            except AttributeError:
                if self.debug:
                    print ("Cannot recognize format, skippng parsing of the data")
            else:
                if self.debug:
                    print ("Format recognized, trying to parse data")
                for folder in data.Document.Folder:
                    try:
                        folder_name = str(folder.name)
                    except UnicodeEncodeError:
                        if self.debug:
                            print ('Something is wrong with the name format (contains special characters which \'lxml.objectify.StringElement\' cannot handle); defaulting name to \'NaN\'')
                        folder_name = 'NaN'
                    if folder_name != 'Danmark':
                        for element in folder.Placemark:
                            # extract the coordinates so a test can be made on the size due to a bug in Kjelds no-fly zone file
                            coordinates_raw =  element.Polygon.outerBoundaryIs.LinearRing.coordinates
                            coordinates_str_split = str(coordinates_raw).split(' ')
                            del coordinates_str_split[-1] # delete the last element since it is just empty and not 3 elements

                            if len(coordinates_str_split) > 2:
                                element_style = str(element.styleUrl)
                                element_style = element_style[1:]
                                element_id = str(itr)
                                itr += 1

                                try:
                                    element_name = str(element.name)
                                except UnicodeEncodeError:
                                    if self.debug:
                                        print ('Something is wrong with the name format (contains special characters which \'lxml.objectify.StringElement\' cannot handle); defaulting name to \'NaN\'')
                                    element_name = 'NaN'

                                coordinate3d_placemark = []
                                for coordinate in coordinates_str_split:
                                    coordinate3d = []
                                    sub_coordinates = coordinate.split(',')
                                    for sub_coordinate in sub_coordinates:
                                        coordinate3d.append(float(sub_coordinate))
                                    coordinate3d = [coordinate3d[1], coordinate3d[0], coordinate3d[2]]  # correct the order to lat, lon, alt_rel instead of lon, lat, alt_rel
                                    coordinate3d_placemark.append(coordinate3d)
                                if self.debug:
                                    print (folder_name, element_name, element_style, element_id, len(coordinates_str_split))
                                self.coordinate3d_combined.append({'style': element_style, 'id': element_id, 'name': element_name, 'coordinates': coordinate3d_placemark})
                    else:
                        if self.debug:
                            print ("Found denmark folder, skipping zones")
        else:
            for element in data.Document.Placemark:
                itr += 1
                if self.debug:
                    print ('\nPlacemark', itr)
                try:
                    element.Polygon
                except AttributeError:
                    if self.debug:
                        print ('Something is wrong about the zone formatting; trying different method')
                    try:
                        element.MultiGeometry
                    except AttributeError:
                        if self.debug:
                            print ('Something is wrong about the zone formatting; skipping zone!')
                        continue
                    else:
                        if self.debug:
                            print ('MultiGeometry object, amount of subpolygons', len(element.MultiGeometry.Polygon))

                        # Save the values that are the same for each subpolygon
                        element_style = str(element.styleUrl)
                        element_style = element_style[9:]
                        element_id = str(element.id)
                        try:
                            element_name = str(element.name)
                        except UnicodeEncodeError:
                            if self.debug:
                                print ('Something is wrong with the name format (contains special characters which \'lxml.objectify.StringElement\' cannot handle); defaulting name to \'NaN\'')
                            element_name = 'NaN'
                        if element_name != 'Danmark':
                            sub_itr = 0
                            for sub_element in element.MultiGeometry.Polygon:
                                coordinates_raw =  sub_element.outerBoundaryIs.LinearRing.coordinates
                                coordinates_str_split = str(coordinates_raw).split(' ')
                                del coordinates_str_split[-1] # delete the last element since it is just empty and not 3 elements

                                coordinate3d_placemark = []
                                for coordinate in coordinates_str_split:
                                    coordinate3d = []
                                    sub_coordinates = coordinate.split(',')
                                    for sub_coordinate in sub_coordinates:
                                        coordinate3d.append(float(sub_coordinate))
                                    coordinate3d = [coordinate3d[1], coordinate3d[0], coordinate3d[2]] # correct the order to lat, lon, alt_rel instead of lon, lat, alt_rel
                                    coordinate3d_placemark.append(coordinate3d)
                                if self.debug:
                                    print (element_id+'-'+str(sub_itr))
                                self.coordinate3d_combined.append({'style': element_style, 'id': element_id+'-'+str(sub_itr), 'name': element_name, 'coordinates': coordinate3d_placemark})
                                sub_itr += 1
                else:
                    if self.debug:
                        print ('Single Polygon object')
                    element_style = str(element.styleUrl)
                    element_style = element_style[9:]
                    element_id = str(element.id)
                    try:
                        element_name = str(element.name)
                    except UnicodeEncodeError:
                        if self.debug:
                            print ('Something is wrong with the name format (contains special characters which \'lxml.objectify.StringElement\' cannot handle); defaulting name to \'NaN\'')
                        element_name = 'NaN'
                    if element_name != 'Danmark':
                        coordinates_raw =  element.Polygon.outerBoundaryIs.LinearRing.coordinates
                        coordinates_str_split = str(coordinates_raw).split(' ')
                        del coordinates_str_split[-1] # delete the last element since it is just empty and not 3 elements

                        coordinate3d_placemark = []
                        for coordinate in coordinates_str_split:
                            coordinate3d = []
                            sub_coordinates = coordinate.split(',')
                            for sub_coordinate in sub_coordinates:
                                coordinate3d.append(float(sub_coordinate))
                            coordinate3d = [coordinate3d[1], coordinate3d[0], coordinate3d[2]]  # correct the order to lat, lon, alt_rel instead of lon, lat, alt_rel
                            coordinate3d_placemark.append(coordinate3d)
                        if self.debug:
                            print (element_id)
                        self.coordinate3d_combined.append({'style': element_style, 'id': element_id, 'name': element_name, 'coordinates': coordinate3d_placemark})
            self.placemarks = itr
        if len(self.coordinate3d_combined) == 0:
            print ('No no-fly zones were parsed')
            return False
        else:
            return True

    def print_zones(self):
        """
        Prints all of the no-fly zones
        Input: none
        Output: none but printing in the terminal
        """
        for element in self.coordinate3d_combined:
            self.print_zone(0, element)
    def print_zone(self, zone_no, zone=None):
        """
        Prints a single no-fly zone
        Input: zone number and optional zone (if provided it uses the provided zone instead of adressing a number)
        Output: none but printing in the terminal
        """
        if zone == None:
            print ('\nStyle: %s' % self.coordinate3d_combined[zone_no]['style'])
            print ('ID: %s' % self.coordinate3d_combined[zone_no]['id'])
            print ('Name: %s' % self.coordinate3d_combined[zone_no]['name'])
            for element in self.coordinate3d_combined[zone_no]['coordinates']:
                print ('   '+str(element))
        else:
            print ('\nStyle: %s' % zone['style'])
            print ('ID: %s' % zone['id'])
            print ('Name: %s' % zone['name'])
            for element in zone['coordinates']:
                print ('   '+str(element))

    def get_zone_coordinates(self, zone_no):
        """
        Returns no-fly zone coordinates of the argument provided no-fly zone
        Input: zone number
        Output: no-fly zone coordinates
        """
        if 0 <= zone_no < len(self.coordinate3d_combined):
            return self.coordinate3d_combined[zone_no]['coordinates']
        else:
            return []

    def get_zone_style(self, zone_no):
        """
        Returns no-fly zone style of the argument provided no-fly zone
        Input: zone number
        Output: no-fly zone style
        """
        if 0 <= zone_no < len(self.coordinate3d_combined):
            return self.coordinate3d_combined[zone_no]['style']
        else:
            return ''

    def get_zone_id(self, zone_no):
        """
        Returns no-fly zone ID of the argument provided no-fly zone
        Input: zone number
        Output: no-fly zone ID
        """
        if 0 <= zone_no < len(self.coordinate3d_combined):
            return self.coordinate3d_combined[zone_no]['id']
        else:
            return ''

    def get_zone_name(self, zone_no):
        """
        Returns no-fly zone name of the argument provided no-fly zone (note that if the parsing failed with the name it returns 'NaN')
        Input: zone number
        Output: no-fly zone name
        """
        if 0 <= zone_no < len(self.coordinate3d_combined):
            return self.coordinate3d_combined[zone_no]['name']
        else:
            return []

    def get_zone(self, zone_no):
        """
        Returns a no-fly zone object of the argument provided no-fly zone
        Input: zone number
        Output: no-fly zone
        """
        return self.coordinate3d_combined[zone_no]

    def get_zones(self):
        """
        Returns all the no-fly zones
        Input: none
        Output: no-fly zones
        """
        return self.coordinate3d_combined

    def get_number_of_zones(self):
        """
        Returns the number of no-fly zones
        Input: none
        Output: number of no-fly zones (int)
        """
        return len(self.coordinate3d_combined)
    def get_number_of_placemarks(self):
        """
        Returns the number of placemarks parsed
        Input: none
        Output: number of placemarks (int)
        """
        return self.placemarks

    def get_polygon_index_from_id(self, id):
        """
        Finds the index associated with the ID
        Input: ID of the sought element
        Output: bool (True = ID found, False = ID not found) and index (defaulted to 0 if element not found)
        """
        itr = 0
        for element in self.coordinate3d_combined:
            if element['id'] == id:
                return True, itr
            itr += 1
        return False, 0

if __name__ == '__main__':
    # Run self test
    #test = kml_no_fly_zones_parser()
    test = kml_no_fly_zones_parser(True)

    print ('\n\nParsing KML file')
    # There are 2 options for parsing the data
    #if test.parse_file('KmlUasZones_2018-02-27-18-24.kml'):
    #if test.parse_file('KmlUasZones_sec2.kml'):
    if test.parse_file("export.kml"):
        print ('The KML has successfully been parsed; parsed %i placemarks and %i no-fly zones (includes subzones defined in MultiGeometry objects)' % (test.get_number_of_placemarks(), test.get_number_of_zones()))

        print ('\n\nPrinting parsed KML no-fl zones in 2s')
        time.sleep(2)
        test.print_zones()

        print ('\n\nPrinting selected zone in 2s')
        time.sleep(2)
        test.print_zone(0)

        print ('\n\nPrinting seleced zone fields from acessor methods in 2s')
        time.sleep(2)
        print (test.get_zone(0))
        print (test.get_zone_style(0))
        print (test.get_zone_id(0))
        print (test.get_zone_name(0))
        print (test.get_zone_coordinates(0))
