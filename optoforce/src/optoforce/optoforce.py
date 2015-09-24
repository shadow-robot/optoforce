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
    _sensor_frame_length = {'31': 16,
                            '34': 34,
                            '64': 22}

    # TODO Use Conversion to Newtons (needs sensitivity report) instead  of this simple scaling
    _scale = 10000

    def __init__(self):
        """
        Initialize OptoforceDriver object
        @param name - name of the MoveIt group
        """
        self._serial = serial.Serial('/dev/ttyACM0', 1000000, timeout=0.5)
        self._sensor_type = '34'
        self._publishers = []
        self._wrenches = []

        if self._sensor_type == '34':
            for i in range(4):  # 4 sensors
                self._publishers.append(rospy.Publisher("optoforce_" + str(i), WrenchStamped, queue_size=100))
                wrench = WrenchStamped()
                wrench.header.frame_id = "optoforce_" + str(i)
                self._wrenches.append(wrench)

    def config(self):
        # TODO Send config parameters
        pass

    def run(self):
        """
        Runs the read loop.
        """
        while(True):
            s = self._serial.read(self._sensor_frame_length[self._sensor_type])
            data = self._decode(s)
            if data:
                self._publish(data)
            # print binascii.hexlify(s)

    def _decode(self, frame):
        """
        Decodes a sensor frame
        It assumes that we get an entire frame and nothing else from serial.read. This assumption simplifies the code
        and it seems to be always true for the moment.
        @param frame - byte frame from the sensor
        """
        if not self._is_checksum_valid(frame):
            return None

        data = OptoforceData()
        offset = 6
        data.status = struct.unpack_from('>H', frame, offset)[0]

        if self._sensor_type == '34':
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
        if self._sensor_type == '34':
            for i in range(4):  # 4 sensors
                self._wrenches[i].header.stamp = stamp
                self._wrenches[i].wrench.force.x = data.force[i][0]
                self._wrenches[i].wrench.force.y = data.force[i][1]
                self._wrenches[i].wrench.force.z = data.force[i][2]
                self._publishers[i].publish(self._wrenches[i])

    @staticmethod
    def _is_checksum_valid(frame):
        offset = 0
        calculated = 0

        for _ in range(len(frame) - 2):
            val = struct.unpack_from('>B', frame, offset)[0]
            calculated += val
            offset += 1

        checksum = struct.unpack_from('>H', frame, offset)[0]
        return calculated == checksum

if __name__ == '__main__':
    rospy.init_node("optoforce")
    driver = OptoforceDriver()
    driver.config()
    driver.run()