#!/usr/bin/env python
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

import sys
import requests
import json
from termcolor import colored
import time

# Disable warning
from requests.packages.urllib3.exceptions import InsecureRequestWarning
requests.packages.urllib3.disable_warnings(InsecureRequestWarning)

if __name__ == '__main__':
    payload = {
        'uav_id': 3013,
        'uav_auth_key': '8bdb3673e9429c9f011368d2433558d3860d69975cc830c84e4f86b1b712f18c67ff693a54304ca417f44161bf13f4e2ecd66a7c080889225f2f1f171f6f20c8',
        'uav_op_status': 3,
        'pos_cur_lat_dd': 55.371653,
        'pos_cur_lng_dd': 10.428223,
        'pos_cur_alt_m': 30,
        'pos_cur_hdg_deg': 7,
        'pos_cur_vel_mps': 10,
        'pos_cur_gps_timestamp': 123456,
        'wp_next_lat_dd': 55.371653,
        'wp_next_lng_dd': 10.428223,
        'wp_next_alt_m': 10,
        'wp_next_hdg_deg': 0,
        'wp_next_vel_mps': 5,
        'wp_next_eta_epoch': 1537428516,
        'uav_bat_soc': 100
    }
    #while True:
    print (colored('Trying to POST the data...', 'yellow'))
    r = ''
    try:
        r = requests.post(url = 'https://droneid.dk/rmuasd/utm/tracking_data.php', data = payload, timeout=2)
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
        if r.text == '1': # This check can in theory be omitted since the header check should catch an error
            print (colored('Success!\n', 'green'))
            print (colored('Status code: %i' % r.status_code, 'yellow'))
            print (colored('Content type: %s' % r.headers['content-type'], 'yellow'))
