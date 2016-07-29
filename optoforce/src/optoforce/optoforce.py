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
import array
import serial
import struct
import binascii
import rospy
import std_srvs.srv
from geometry_msgs.msg import WrenchStamped


class OptoforceData:
    def __init__(self):
        self.status = None
        self.force = []


class OptoforceDriver(object):
    """
    Driver for Optoforce sensors
    """
    _OPTOFORCE_TYPE_31 = 0
    _OPTOFORCE_TYPE_34 = 1
    _OPTOFORCE_TYPE_64 = 2

    _sensor_frame_length = {_OPTOFORCE_TYPE_31: 16,
                            _OPTOFORCE_TYPE_34: 34,
                            _OPTOFORCE_TYPE_64: 22}

    _daq_type_map = {"s-ch/3-axis": _OPTOFORCE_TYPE_31,
                     "m-ch/3-axis": _OPTOFORCE_TYPE_34,
                     "s-ch/6-axis": _OPTOFORCE_TYPE_64}

    _config_response_frame_length = 7

    _speed_values = {"Stop":    0,
                     "1000Hz": 1,
                     "333Hz": 3,
                     "100Hz": 10,
                     "30Hz": 33,
                     "10Hz": 100}

    _filter_values = {"No": 0,
                      "500Hz": 1,
                      "150Hz": 2,
                      "50Hz": 3,
                      "15Hz": 4,
                      "5Hz": 5,
                      "1.5Hz": 6}

    _zeroing_values = {False: 0,
                       True: 255}

    def __init__(self):
        """
        Initialize OptoforceDriver object
        """
        try:
            port = rospy.get_param("~port", "/dev/ttyACM0")
            self._serial = serial.Serial(port, 1000000, timeout=None)
        except serial.SerialException as e:
            rospy.logfatal(e.message)
            rospy.signal_shutdown("Serial connection failure")
            sys.exit(1)
        sensor_type_param = rospy.get_param("~type", "m-ch/3-axis")
        self._sensor_type = self._daq_type_map[sensor_type_param]
        self._starting_index = rospy.get_param("~starting_index", 0)
        self._publishers = []
        self._wrenches = []
        self._nb_sensors = 0
        self._nb_axis = 0

        if self._sensor_type == self._OPTOFORCE_TYPE_31:
            self._nb_sensors = 1
            self._nb_axis = 3
        elif self._sensor_type == self._OPTOFORCE_TYPE_34:
            self._nb_sensors = 4
            self._nb_axis = 3
        elif self._sensor_type == self._OPTOFORCE_TYPE_64:
            self._nb_sensors = 1
            self._nb_axis = 6

        self._scale = rospy.get_param("~scale")

        if len(self._scale) != self._nb_sensors:
            raise ValueError("Number of sensors [%i]and scaling factor vectors [%i] given doesn't match." % (self._nb_sensors, len(self._scale)))
        else:
            for x in range(self._nb_sensors):
                if len(self._scale[x]) != self._nb_axis:
                    raise ValueError("Number of axis [%i] and scaling factors [%i] given doesn't match." % (self._nb_axis, len(self._scale[x])))

        for i in range(self._nb_sensors):
            self._publishers.append(rospy.Publisher("optoforce_" + str(self._starting_index + i), WrenchStamped,
                                                    queue_size=100))
            wrench = WrenchStamped()
            wrench.header.frame_id = "optoforce_" + str(self._starting_index + i)
            self._wrenches.append(wrench)

        # Advertise a service to retrieve the unique ID of the sensor
        self._service = rospy.Service('get_unique_id', std_srvs.srv.Empty, self.get_unique_id)

        # Tree representation of all accepted headers.
        # Leaf nodes are the lenght of the frame associated with the header
        self._headers = {
            170:
                {0:
                    {18:
                        {8: 14}},
                7:
                    {8:
                        {10: 16,
                        16: 34,
                        28: 22}}}}
        # self._headers = {
        #     struct.pack('>B', 171):
        #         {struct.pack('>B', 0):
        #             {struct.pack('>B', 18):
        #                 {struct.pack('>B', 8): 14}}},
        #     struct.pack('>B', 170):
        #         {struct.pack('>B', 7):
        #             {struct.pack('>B', 8):
        #                 {struct.pack('>B', 10): 16,
        #                 struct.pack('>B', 16): 34,
        #                 struct.pack('>B', 28): 22}}}}

    def config(self):
        speed = self._speed_values[rospy.get_param("~speed", "100Hz")]
        filter = self._filter_values[rospy.get_param("~filter", "15Hz")]
        zero = self._zeroing_values[rospy.get_param("~zero", "false")]
        config_length = 9

        header = struct.pack('>4B', 170, 0, 50, 3)
        offset = 0

        frame = array.array('B', [0] * config_length)
        struct.pack_into('>4s3B', frame, offset, header, speed, filter, zero)

        checksum = self._checksum(frame, len(frame))
        offset = len(header) + 3
        struct.pack_into('>H', frame, offset, checksum)
        rospy.logdebug(binascii.hexlify(frame))

        self._serial.write(frame)

    def get_unique_id(self, request):
        config_length = 6
        offset = 0

        # Build the frame and send it
        frame = array.array('B', [0] * config_length)
        struct.pack_into('>6B', frame, offset, 171, 0, 18, 8, 0, 197)
        self._serial.write(frame)

        return std_srvs.srv.EmptyResponse()

    def run(self):
        """
        Runs the read loop.
        """
        sizes_seen = []

        while not rospy.is_shutdown():
            data = self._detect_header(self._headers)
            if len(data) > 1 and len(data) not in sizes_seen:
                sizes_seen.append(len(data))
                format_string = '>' + str(len(data)) + 'B'
                rospy.logdebug(format_string)
                rospy.logdebug("frame s: " + str(struct.unpack(format_string, data)))

            if len(data) > 1:
                self._decode(data)
            else:
                rospy.logwarn("frame with unknown header received")

            # s = self._serial.read(self._sensor_frame_length[self._sensor_type])
            # rospy.logdebug(binascii.hexlify(s))
            # data = self._decode(s)
            # if data:
            #     self._publish(data)

    def _detect_header(self, tree):
        raw_byte = self._serial.read()
        byte = struct.unpack('>B', raw_byte)[0]

        for header_byte, subtree in tree.items():
            if byte == header_byte:
                if type(subtree) is int:
                    return raw_byte + self._serial.read(subtree-4)
                else:
                    next_bytes = self._detect_header(subtree)
                    if next_bytes: # hacky
                        return raw_byte + next_bytes
                    else:
                        return '' # hacky
        return '' # hacky

    def _decode(self, frame):
        """
        Decodes a sensor frame
        It assumes that we get an entire frame and nothing else from serial.read. This assumption simplifies the code
        and it seems to be always true for the moment.
        @param frame - byte frame from the sensor
        """
        if not self._is_checksum_valid(frame):
            rospy.logwarn("Bad checksum in frame:\n" + frame)
            # This is a trick to recover the frame synchronisation without having to implement a state machine.
            # We are assuming that the reception of a config response frame (of shorter length) is the cause
            # of the loss of frame synchronisation
            # This method would be wrong if the cause were an actual transmission error, but this doesn't
            # seem to happen.
            # FIXME: s = self._serial.read(self._config_response_frame_length)
            return None

        header = struct.unpack_from('>4B', frame)

        data_headers = [(170, 7, 8, 10), (170, 7, 8, 16), (170, 7, 8, 28)]
        if header in data_headers:
            data = OptoforceData()
            offset = 6
            data.status = struct.unpack_from('>H', frame, offset)[0]

            for s in range(self._nb_sensors):
                force_axes = []
                for a in range(self._nb_axis):
                    offset += 2
                    val = struct.unpack_from('>h', frame, offset)[0]
                    # TODO Convert to Newtons (needs sensitivity report)
                    val = float(val) / self._scale[s][a]
                    force_axes.append(val)
                data.force.append(force_axes)

            return data
        elif header == (170, 0, 18, 8):
            rospy.logdebug("header match for unique identifier!")
            offset = 4
            string = struct.unpack_from('>8c', frame, offset)
            print(''.join(string))
        else:
            rospy.logwarn("I can't recognize the frame's header:\n" + frame)
            return None

    def _publish(self, data):
        stamp = rospy.Time.now()
        for i in range(self._nb_sensors):
            self._wrenches[i].header.stamp = stamp
            self._wrenches[i].wrench.force.x = data.force[i][0]
            self._wrenches[i].wrench.force.y = data.force[i][1]
            self._wrenches[i].wrench.force.z = data.force[i][2]
            if self._nb_axis == 6:
                self._wrenches[i].wrench.torque.x = data.force[i][3]
                self._wrenches[i].wrench.torque.y = data.force[i][4]
                self._wrenches[i].wrench.torque.z = data.force[i][5]
            self._publishers[i].publish(self._wrenches[i])

    @staticmethod
    def _checksum(frame, length):
        offset = 0
        calculated = 0

        for _ in range(length):
            val = struct.unpack_from('>B', frame, offset)[0]
            calculated += val
            offset += 1

        return calculated

    @classmethod
    def _is_checksum_valid(cls, frame):
        calculated = cls._checksum(frame, len(frame) - 2)
        offset = len(frame) - 2

        checksum = struct.unpack_from('>H', frame, offset)[0]
        return calculated == checksum

if __name__ == '__main__':
    rospy.init_node("optoforce", log_level=rospy.DEBUG)
    driver = OptoforceDriver()
    driver.config()
    driver.run()
    # rospy.spin()
