#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Descriptors: TL = Tobias Lundby (tolu@mmmi.sdu.dk)
2018-09-20 TL Created file
"""

"""
Description:
Example of how to get tehcnical data for a UAV using the 'Get UAV technical data' API
License: BSD 3-Clause
"""

import rospy
import geometry_msgs.msg
import std_msgs.msg

import sys
import requests
import json
from termcolor import colored

# Disable warning
from requests.packages.urllib3.exceptions import InsecureRequestWarning
requests.packages.urllib3.disable_warnings(InsecureRequestWarning)

if __name__ == '__main__':
    payload = {
        'uav_id': 3013
    }
    r = ''
    try:
        r = requests.get(url = 'https://droneid.dk/rmuasd/utm/tracking_data.php', timeout=500)
        r.raise_for_status()
    except requests.exceptions.Timeout:
	    # Maybe set up for a retry, or continue in a retry loop
        print (colored('Request has timed out', 'red'))
    except requests.exceptions.TooManyRedirects:
        # Tell the user their URL was bad and try a different one
        print (colored('Request has too many redirects', 'red'))
    except requests.exceptions.HTTPError as err:
        print (colored('HTTP error', 'red'))
        print (colored(err, 'yellow'))
        #sys.exit(1) # Consider the exit since it might be unintentional in some cases
    except requests.exceptions.RequestException as err:
        # Catastrophic error; bail.
        print (colored('Request error', 'red'))
        print (colored(err, 'yellow'))
        sys.exit(1)
    else:
        print (colored('Status code: %i' % r.status_code, 'yellow'))
        print (colored('Content type: %s' % r.headers['content-type'], 'yellow'))

        data_dict = ''
        try:
            data_dict = json.loads(r.text) # convert to json
        except:
            print (colored('Error in parsing of data to JSON', 'red'))
        else:
            #print r.text # Print the raw body data
            for entry in data_dict: # The loop could be omitted since there should only be 1 entry and the header exception should catch a request for a UAV ID which does not exist.
                print( 'UAV ID: %i, name: %s, weight: %f, max. vertical velocity: %f, max. endurance: %f' % (entry['uav_id'], entry['uav_op_status'], entry['uav_bat_soc'], entry['time_epoch'], entry['pos_cur_lat_dd']) )

# if __name__ == '__main__':
#     payload = {
#         'uav_id': 3013
#     }
#     r = ''
#     try:
#         r = requests.get(url = 'https://droneid.dk/rmuasd/utm/uav.php', params = payload, timeout=2)
#         r.raise_for_status()
#     except requests.exceptions.Timeout:
# 	    # Maybe set up for a retry, or continue in a retry loop
#         print (colored('Request has timed out', 'red'))
#     except requests.exceptions.TooManyRedirects:
#         # Tell the user their URL was bad and try a different one
#         print (colored('Request has too many redirects', 'red'))
#     except requests.exceptions.HTTPError as err:
#         print (colored('HTTP error', 'red'))
#         print (colored(err, 'yellow'))
#         #sys.exit(1) # Consider the exit since it might be unintentional in some cases
#     except requests.exceptions.RequestException as err:
#         # Catastrophic error; bail.
#         print (colored('Request error', 'red'))
#         print (colored(err, 'yellow'))
#         sys.exit(1)
#     else:
#         print (colored('Status code: %i' % r.status_code, 'yellow'))
#         print (colored('Content type: %s' % r.headers['content-type'], 'yellow'))
#
#         data_dict = ''
#         try:
#             data_dict = json.loads(r.text) # convert to json
#         except:
#             print (colored('Error in parsing of data to JSON', 'red'))
#         else:
#             #print r.text # Print the raw body data
#             for entry in data_dict: # The loop could be omitted since there should only be 1 entry and the header exception should catch a request for a UAV ID which does not exist.
#                 print( 'UAV ID: %i, name: %s, weight: %f, max. vertical velocity: %f, max. endurance: %f' % (entry['uav_id'], entry['uav_name'], entry['uav_weight_kg'], entry['uav_max_vel_mps'], entry['uav_max_endurance_s']) )
