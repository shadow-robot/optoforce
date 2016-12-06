#!/usr/bin/env python

# Copyright 2015 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.

import sys
import signal
import optoforce
import logging

if __name__ == '__main__':
    class signal_handler:
        def __init__(self):
            self.stop = False
        def __call__(self, signal, frame):
            self.stop = True

    signal_handle = signal_handler()
    signal.signal(signal.SIGINT, signal_handle)

    # # Configure logging
    #
    # # create logger
    # logger = logging.getLogger()
    # logger.setLevel(logging.DEBUG)
    #
    # # # create console handler and set level to debug
    # # ch = logging.StreamHandler()
    # # ch.setLevel(logging.DEBUG)
    #
    # # create file handler and set level to debug
    # ch = logging.FileHandler('udev-optoforce.log')
    # ch.setLevel(logging.DEBUG)
    #
    # # create formatter
    # formatter = logging.Formatter('%(asctime)s :: %(name)s :: %(levelname)s :: %(message)s')
    #
    # # add formatter to ch
    # ch.setFormatter(formatter)
    #
    # # add ch to logger
    # logger.addHandler(ch)

    if len(sys.argv) > 1:
        port = "/dev/" + sys.argv[1]
        sensor_type = "s-ch/3-axis"
        scale = [[100, 100, 100]]
        driver = optoforce.OptoforceDriver(port, sensor_type, scale)

        driver.request_serial_number()
        while not signal_handle.stop:
            try:
                data = driver.read()

                if isinstance(data, optoforce.OptoforceData):
                    print '.',
                elif isinstance(data, optoforce.OptoforceSerialNumber):
                    print("optoforce_" + str(data))
                    sys.exit(0)
            except optoforce.OptoforceError:
                pass

    import random as r
    r.seed()
    print "optoforce_" + str(r.random())
