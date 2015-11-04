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

import array
import serial
import struct
import binascii
import rospy
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

    # TODO Use Conversion to Newtons (needs sensitivity report) instead  of this simple scaling
    _scale = 10000

    def __init__(self):
        """
        Initialize OptoforceDriver object
        """
        port = rospy.get_param("~port", "/dev/ttyACM0")
        self._serial = serial.Serial(port, 1000000, timeout=None)
        self._sensor_type = self._OPTOFORCE_TYPE_34
        self._publishers = []
        self._wrenches = []

        if self._sensor_type == self._OPTOFORCE_TYPE_34:
            for i in range(4):  # 4 sensors
                self._publishers.append(rospy.Publisher("optoforce_" + str(i), WrenchStamped, queue_size=100))
                wrench = WrenchStamped()
                wrench.header.frame_id = "optoforce_" + str(i)
                self._wrenches.append(wrench)

    def config(self):
        speed = self._speed_values[rospy.get_param("~speed", "100Hz")]
        filter = self._filter_values[rospy.get_param("~filter", "15Hz")]
        zero = self._zeroing_values[rospy.get_param("~zero", "false")]
        config_length = 9

        header = self._frame_header()
        offset = 0

        frame = array.array('B', [0] * config_length)
        struct.pack_into('>4s3B', frame, offset, header, speed, filter, zero)

        checksum = self._checksum(frame, len(frame))
        offset = len(header) + 3
        struct.pack_into('>H', frame, offset, checksum)
        rospy.logdebug(binascii.hexlify(frame))

        self._serial.write(frame)

    def run(self):
        """
        Runs the read loop.
        """
        while not rospy.is_shutdown():
            s = self._serial.read(self._sensor_frame_length[self._sensor_type])
            rospy.logdebug(binascii.hexlify(s))
            data = self._decode(s)
            if data:
                self._publish(data)

    def _decode(self, frame):
        """
        Decodes a sensor frame
        It assumes that we get an entire frame and nothing else from serial.read. This assumption simplifies the code
        and it seems to be always true for the moment.
        @param frame - byte frame from the sensor
        """
        if not self._is_checksum_valid(frame):
            rospy.logwarn("Bad checksum")
            # This is a trick to recover the frame synchronisation without having to implement a state machine.
            # We are assuming that the reception of a config response frame (of shorter length) is the cause
            # of the loss of frame synchronisation
            # This method would be wrong if the cause were an actual transmission error, but this doesn't
            # seem to happen.
            s = self._serial.read(self._config_response_frame_length)
            return None

        data = OptoforceData()
        offset = 6
        data.status = struct.unpack_from('>H', frame, offset)[0]

        if self._sensor_type == self._OPTOFORCE_TYPE_34:
            for _ in range(4):  # 4 sensors
                force_axes = []
                for __ in range(3):  # 3 axes per sensor
                    offset += 2
                    val = struct.unpack_from('>h', frame, offset)[0]
                    # TODO Convert to Newtons (needs sensitivity report)
                    val = float(val) / self._scale
                    force_axes.append(val)
                data.force.append(force_axes)
        return data

    def _publish(self, data):
        stamp = rospy.Time.now()
        if self._sensor_type == self._OPTOFORCE_TYPE_34:
            for i in range(4):  # 4 sensors
                self._wrenches[i].header.stamp = stamp
                self._wrenches[i].wrench.force.x = data.force[i][0]
                self._wrenches[i].wrench.force.y = data.force[i][1]
                self._wrenches[i].wrench.force.z = data.force[i][2]
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

    @staticmethod
    def _frame_header():
        return struct.pack('>4B', 170, 0, 50, 3)


if __name__ == '__main__':
    rospy.init_node("optoforce")
    driver = OptoforceDriver()
    driver.config()
    driver.run()
