#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Descriptors: TL = Tobias Lundby (tolu@mmmi.sdu.dk)
2018-09-21 TL Created file
"""

"""
Description:
Example of how to register a UAV and operator using the 'Register UAV' API in order to get a UAV ID and authentication key.
License: BSD 3-Clause
"""

import rospy
import geometry_msgs.msg
import std_msgs.msg

from std_msgs.msg import Bool
from std_msgs.msg import Int64

from utm.msg import utm_tracking_data
from utm.msg import utm_no_flight_circle
from utm.msg import utm_no_flight_area
from utm.msg import utm_rally_point
from utm.msg import utm_rally_point_list

from lora_ground_control.msg import heartbeat_node

import sys
import requests
import json
from termcolor import colored
import time

import kml_reader

# Disable warning
from requests.packages.urllib3.exceptions import InsecureRequestWarning
requests.packages.urllib3.disable_warnings(InsecureRequestWarning)

uav_id = 3013
# We expect to have one heartbeat at least every second. We often publisher a bit faster
# to ensure we uphold the timing
HEARTBEAT_PERIOD = 0.5
HEARTBEAT_PERIOD_EXPECTED = 1

DATA_FETCH_PERIOD = 0.5
DATA_FETCH_PERIOD_EXPECTED = 1

class UTM_node:

    def __init__(self):

        rospy.init_node('UTM_node', anonymous=True)

        # heartbeat last sent
        self.last_heartbeat = rospy.get_time()
        self.last_tracking_fetch = rospy.get_time()

        self.id = 10
        self.name = "UTM"
        self.current_task = "Idle"
        self.main_status = "Running"
        self.has_error = False


        # Subscribers
        rospy.Subscriber("utm/add_tracking_data",
                         utm_tracking_data,
                         self.add_tracking_data_callback,
                         queue_size=1)

        rospy.Subscriber("utm/request_no_flight_zones",
                         Bool,
                         self.request_no_flight_zones_callback,
                         queue_size=1)

        rospy.Subscriber("utm/request_utm_is_up",
                      Bool,
                      self.request_utm_is_up_callback,
                      queue_size=1)

        rospy.Subscriber("utm/request_rally_points",
                    Bool,
                    self.request_rally_points,
                    queue_size=1)

        # Publishers
        self.tracking_data_pub = rospy.Publisher(
            "utm/fetch_tracking_data",
            utm_tracking_data,
            queue_size=1000)

        self.number_of_zones_pub = rospy.Publisher(
            "utm/number_of_zones",
            Int64,
            queue_size=1)

        self.no_flight_areas_pub = rospy.Publisher(
            "utm/fetch_no_flight_areas",
            utm_no_flight_area,
            queue_size=1000)

        self.no_flight_circles_pub = rospy.Publisher(
            "utm/fetch_no_flight_circles",
            utm_no_flight_circle,
            queue_size=1000)

        self.utm_is_up_pub = rospy.Publisher(
            "utm/utm_is_up",
            Bool,
            queue_size=1)

        self.rally_points_pub = rospy.Publisher(
            "utm/fetch_rally_points",
            utm_rally_point_list,
            queue_size=1)

        self.heartbeat_pub = rospy.Publisher(
            "/lora_ground_control/heartbeat_nodes_rx",
            heartbeat_node,
            queue_size=1)

        self.total_number_of_zones = 0



    def run(self):
        rospy.sleep(1)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.last_tracking_fetch + rospy.rostime.Duration.from_sec(DATA_FETCH_PERIOD).secs <= rospy.get_time():
                self.fetch_tracking_data()

            if self.last_heartbeat + rospy.rostime.Duration.from_sec(HEARTBEAT_PERIOD).secs <= rospy.get_time():
                self.send_heartbeat()

            rate.sleep()

        self.main_status = "Shutting down"
        return

    def send_heartbeat(self):
        # make and send heartbeat
        hb = heartbeat_node()
        hb.current_task = self.current_task
        hb.main_status = self.main_status
        hb.expected_interval = HEARTBEAT_PERIOD_EXPECTED
        hb.has_error = self.has_error
        hb.id = self.id
        hb.name = self.name

        self.heartbeat_pub.publish(hb)
        self.last_heartbeat = rospy.get_time()


    def connect_to_utm_get(self, url, payload):
        r = ''
        succes = False
        try:
            r = requests.get(url=url, params=payload, timeout=2)
            r.raise_for_status()
        except requests.exceptions.Timeout:
            # Maybe set up for a retry, or continue in a retry loop
            print(colored('Request has timed out', 'red'))
        except requests.exceptions.TooManyRedirects:
            # Tell the user their URL was bad and try a different one
            print(colored('Request has too many redirects', 'red'))
        except requests.exceptions.HTTPError as err:
            print(colored('HTTP error', 'red'))
            print(colored(err, 'yellow'))
            # sys.exit(1) # Consider the exit since it might be unintentional in some cases
        except requests.exceptions.RequestException as err:
            # Catastrophic error; bail.
            print(colored('Request error', 'red'))
            print(colored(err, 'yellow'))
            #sys.exit(1)
        else:
            if r.status_code == 204:
                print(colored('No data available. Status code: %i' % r.status_code, 'magenta'))
                succes = False
            else:
                succes = True
                print(colored('Status code: %i' % r.status_code, 'yellow'))
                print(colored('Content type: %s' % r.headers['content-type'], 'yellow'))

        return succes, r

    def connect_to_utm_post(self, url, payload):
        r = ''
        try:
            r = requests.post(url=url, data=payload, timeout=2)
            r.raise_for_status()
        except requests.exceptions.Timeout:
            # Maybe set up for a retry, or continue in a retry loop
            print(colored('Request has timed out', 'red'))
        except requests.exceptions.TooManyRedirects:
            # Tell the user their URL was bad and try a different one
            print(colored('Request has too many redirects', 'red'))
        except requests.exceptions.HTTPError as err:
            print(colored('HTTP error', 'red'))
            print(colored(err, 'yellow'))
            # sys.exit(1) # Consider the exit since it might be unintentional in some cases
        except requests.exceptions.RequestException as err:
            # Catastrophic error; bail.
            print(colored('Request error', 'red'))
            print(colored(err, 'yellow'))
            sys.exit(1)
        else:
            return True, r

        return False, r

    def request_utm_is_up_callback(self, msg):
        newMsg = Bool()
        self.utm_is_up_pub.publish(newMsg)

    def request_rally_points(self, msg):
        self.current_task = "Loading rally points"
        self.fetch_rally_points()
        self.current_task = "Idle"

    def add_tracking_data_callback(self, msg):
        rospy.loginfo("here")
        self.current_task = "Posting UAV tracking data"
        payload = {
            'uav_id': uav_id,
            'uav_auth_key': '8bdb3673e9429c9f011368d2433558d3860d69975cc830c84e4f86b1b712f18c67ff693a54304ca417f44161bf13f4e2ecd66a7c080889225f2f1f171f6f20c8',
            'uav_op_status': msg.uav_op_status,
            'pos_cur_lat_dd': msg.pos_cur_lat_dd,
            'pos_cur_lng_dd': msg.pos_cur_lng_dd,
            'pos_cur_alt_m': msg.pos_cur_alt_m,
            'pos_cur_hdg_deg': msg.pos_cur_hdg_deg,
            'pos_cur_vel_mps': msg.pos_cur_vel_mps,
            'pos_cur_gps_timestamp': msg.pos_cur_gps_timestamp,
            'wp_next_lat_dd': msg.wp_next_lat_dd,
            'wp_next_lng_dd': msg.wp_next_lng_dd,
            'wp_next_alt_m': msg.wp_next_alt_m,
            'wp_next_hdg_deg': msg.wp_next_hdg_deg,
            'wp_next_vel_mps': msg.wp_next_vel_mps,
            'wp_next_eta_epoch': msg.wp_next_eta_epoch,
            'uav_bat_soc': msg.uav_bat_soc
        }

        print(colored('Trying to POST the data...', 'yellow'))

        url = 'https://droneid.dk/rmuasd/utm/tracking_data.php'

        success, r = self.connect_to_utm_post(url, payload)

        if success:
            if r.text == '1':  # This check can in theory be omitted since the header check should catch an error
                print(colored('Success!\n', 'green'))
                print(colored('Status code: %i' % r.status_code, 'yellow'))
                print(colored('Content type: %s' % r.headers['content-type'], 'yellow'))

        self.current_task = "Idle"

    def request_no_flight_zones_callback(self, msg):
        include_static_zones = msg.data

        self.number_of_zones = 0

        if include_static_zones:
            self.current_task = "Loading static no fly zones"
            self.fetch_static_no_fly_zones()

        self.current_task = "Loading dynamic no fly zones"
        self.fetch_dynamic_no_fly_zones()

        self.number_of_zones_pub.publish(self.number_of_zones)
        print("Number of zones: ", self.number_of_zones)
        self.current_task = "Idle"

    def fetch_tracking_data(self):
        self.current_task = "Loading UAV tracking data"
        payload = {
            'time_delta_s': 180
        }

        url = 'https://droneid.dk/rmuasd/utm/tracking_data.php'

        print("Fetching tracking data")
        success, r = self.connect_to_utm_get(url, payload)

        if success:

            data_dict = ''
            try:
                data_dict = json.loads(r.text)  # convert to json
            except:
                print(colored('Error in parsing of data to JSON', 'red'))
            else:
                # print r.text # Print the raw body data
                # already_tracked_uavs = [uav_id]
                already_tracked_uavs = [] # Change this back later! Now tracks ourselves
                for entry in data_dict:

                    if(entry['uav_id'] in already_tracked_uavs):
                        #print(entry['uav_id'], " already exists!")
                        continue

                    already_tracked_uavs.append(entry['uav_id'])

                    msg = utm_tracking_data()

                    msg.uav_id = entry['uav_id']
                    msg.uav_op_status = entry['uav_op_status']
                    msg.pos_cur_lat_dd = entry['pos_cur_lat_dd']
                    msg.pos_cur_lng_dd = entry['pos_cur_lng_dd']
                    msg.pos_cur_alt_m = entry['pos_cur_alt_m']
                    msg.pos_cur_hdg_deg = entry['pos_cur_hdg_deg']
                    msg.pos_cur_vel_mps = entry['pos_cur_vel_mps']
                    msg.pos_cur_gps_timestamp = entry['pos_cur_gps_timestamp']
                    msg.wp_next_lat_dd = entry['wp_next_lat_dd']
                    msg.wp_next_lng_dd = entry['wp_next_lng_dd']
                    msg.wp_next_alt_m = entry['wp_next_alt_m']
                    msg.wp_next_hdg_deg = entry['wp_next_hdg_deg']
                    msg.wp_next_vel_mps = entry['wp_next_vel_mps']
                    msg.wp_next_eta_epoch = entry['wp_next_eta_epoch']
                    msg.uav_bat_soc = entry['uav_bat_soc']

                    self.tracking_data_pub.publish(msg)

        self.last_tracking_fetch = rospy.get_time()
        self.current_task = "Idle"

    def fetch_static_no_fly_zones(self):
        payload = {
            'data_type': 'static_no_fly'
        }

        url = 'https://droneid.dk/rmuasd/utm/data.php'

        print("Fetching static no fly zones")
        success, r = self.connect_to_utm_get(url, payload)

        if success:

            f = open('kml_test.kml', 'w')
            text = r.text[:-7] # For some reason 7 digits are added to the end of the file. They must be removed
            f.write(text)

            reader = kml_reader.kml_no_fly_zones_parser()

            try:
                reader.parse_file('kml_test.kml')
            except:
                print(colored('Error in parsing of data to KML', 'red'))
            else:
                number_of_zones = reader.get_number_of_zones()

                for i in range(number_of_zones):
                    self.number_of_zones = self.number_of_zones + 1

                    msg = utm_no_flight_area()

                    msg.id = i #reader.get_zone_id(i)
                    msg.name = reader.get_zone_name(i)
                    coordinates = reader.get_zone_coordinates(i)

                    msg_coordinates = []
                    for j in coordinates:
                        msg_coordinates.append(j[0])
                        msg_coordinates.append(j[1])
                    msg.polygonCoordinates = msg_coordinates

                    msg.epochValidFrom = -1
                    msg.epochValidTo = -1

                    #print(msg.id)
                    #print(msg.name)
                    #print(msg.polygonCoordinates)
                    #print(msg.epochValidFrom)
                    #print(msg.epochValidTo)

                    #print("SEND")
                    self.no_flight_areas_pub.publish(msg)


    def fetch_dynamic_no_fly_zones(self):
        payload = {
            'data_type': 'dynamic_no_fly'
        }

        url = 'https://droneid.dk/rmuasd/utm/data.php'

        success, r = self.connect_to_utm_get(url, payload)

        print("Fetching dynamic no flight zones ", success)

        if success:

            data_dict = ''
            try:
                data_dict = json.loads(r.text)  # convert to json
            except:
                print(colored('Error in parsing of data to JSON', 'red'))
            else:
                for entry in data_dict:
                    self.number_of_zones = self.number_of_zones + 1

                    zone_type = entry['geometry']

                    if zone_type == "circle":
                        msg = utm_no_flight_circle()

                        msg.id = int(entry['int_id'])
                        msg.name = entry['name']

                        coordinate = entry['coordinates']
                        coordinate = coordinate.split(',')

                        msg.lat = float(coordinate[1])
                        msg.lon = float(coordinate[0])
                        msg.radius = float(coordinate[2])

                        msg.epochValidFrom = int(entry['valid_from_epoch'])
                        msg.epochValidTo = int(entry['valid_to_epoch'])

                        self.no_flight_circles_pub.publish(msg)

                    elif zone_type == "polygon":
                        msg = utm_no_flight_area()

                        msg.id = int(entry['int_id'])
                        msg.name = entry['name']

                        msg.epochValidFrom = int(entry['valid_from_epoch'])
                        msg.epochValidTo = int(entry['valid_to_epoch'])

                        coordinates = entry['coordinates']
                        coordinates = coordinates.split(' ')

                        msg_coordinates = []
                        for coord in coordinates:
                            split_coordinate = coord.split(',')
                            msg_coordinates.append(float(split_coordinate[1]))
                            msg_coordinates.append(float(split_coordinate[0]))

                        msg.polygonCoordinates = msg_coordinates

                        self.no_flight_areas_pub.publish(msg)

    def fetch_rally_points(self):
        payload = {
            'data_type': 'rally_points'
        }

        url = 'https://droneid.dk/rmuasd/utm/data.php'

        print("Fetching rally points")
        success, r = self.connect_to_utm_get(url, payload)

        if success:

            data_dict = ''
            try:
                data_dict = json.loads(r.text)  # convert to json
            except:
                print(colored('fetch_rally_points: Error in parsing of data to JSON', 'red'))
            else:
                msg_list = utm_rally_point_list()
                for entry in data_dict:

                    print(entry)

                    msg = utm_rally_point()

                    msg.int_id = int(entry['int_id'])
                    msg.name = entry['name']

                    msg.lat_dd = float(entry['lat_dd'])
                    msg.lng_dd = float(entry['lng_dd'])

                    msg.alt_rel_m = float(entry['alt_rel_m'])
                    msg.safe_radius_m = float(entry['safe_radius_m'])

                    msg_list.rally_point_list.append(msg)
                    #self.rally_points_pub.publish(msg)

                self.rally_points_pub.publish(msg_list)


if __name__ == '__main__':
    utm_node = UTM_node()
    #utm_node.fetch_tracking_data()
    #utm_node.fetch_static_no_fly_zones()
    #utm_node.fetch_rally_points()
    utm_node.run()
    print("DONE")
