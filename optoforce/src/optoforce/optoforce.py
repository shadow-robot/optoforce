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
import select # used by serial, needed to handle exceptions
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

    # Tree representation of all accepted headers.
    # Leaf nodes are the lenght of the frame associated with the header
    _headers = {170:
                   {0:
                       {18:
                           {8: 14}},
                   7:
                       {8:
                           {10: 16,
                           16: 34,
                       28: 22}}}}

    def __init__(self):
        """
        Initialize OptoforceDriver object
        """
        port = rospy.get_param("~port", "/dev/ttyACM0")
        try:
            self._serial = serial.Serial(port, 1000000, timeout=None)
        except serial.SerialException as e:
            rospy.logfatal(e.message)
            rospy.signal_shutdown("Serial connection failure")
            sys.exit(1)
        sensor_type_param = rospy.get_param("~type", "m-ch/3-axis")
        self._sensor_type = self._daq_type_map[sensor_type_param]
        self._starting_index = rospy.get_param("~starting_index", 0)
        self._append_topic_serial = rospy.get_param("~append_serial_number", "false")
        self._publishers = []
        self._wrenches = []
        self._nb_sensors = 0
        self._nb_axis = 0

        # Set the values for _nb_sensors and _nb_axis based on the sensor at use
        if self._sensor_type == self._OPTOFORCE_TYPE_31:
            self._nb_sensors = 1
            self._nb_axis = 3
        elif self._sensor_type == self._OPTOFORCE_TYPE_34:
            self._nb_sensors = 4
            self._nb_axis = 3
        elif self._sensor_type == self._OPTOFORCE_TYPE_64:
            self._nb_sensors = 1
            self._nb_axis = 6

        # Retrieve and check the scaling factors
        self._scale = rospy.get_param("~scale")

        if len(self._scale) != self._nb_sensors:
            raise ValueError("Number of sensors [%i]and scaling factor vectors "
                "[%i] given doesn't match." % (self._nb_sensors, len(self._scale)))
        else:
            for x in range(self._nb_sensors):
                if len(self._scale[x]) != self._nb_axis:
                    raise ValueError("Number of axis [%i] and scaling factors "
                        "[%i] given doesn't match." % (self._nb_axis, len(self._scale[x])))

        # Create and advertise publishers for each connected sensor
        topic_basename = "optoforce_"
        if self._append_topic_serial:
            serial_number = self.get_serial_number()
            if serial_number:
                topic_basename += serial_number + '_'
                rospy.loginfo("Sensor " + serial_number + " was found at port "
                              + port)
            else:
                rospy.logwarn("Cannot get the serial number from the sensor. "
                           "Falling back to the basinc name scheme")

        for i in range(self._nb_sensors):
            self._publishers.append(rospy.Publisher(topic_basename +
                                                    str(self._starting_index + i),
                                                    WrenchStamped,
                                                    queue_size=100))
            wrench = WrenchStamped()
            wrench.header.frame_id = topic_basename + str(self._starting_index + i)
            self._wrenches.append(wrench)

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
        rospy.logdebug("Sending configuration frame "
                       + self._frame_to_string(frame))

        self._serial.write(frame)

    def get_serial_number(self):
        """
        Ask the sensor for its serial number

        @return the serial number as a string, or None if the request failed
        """
        config_length = 6
        offset = 0

        # Build the request frame and send it
        frame = array.array('B', [0] * config_length)
        struct.pack_into('>6B', frame, offset, 171, 0, 18, 8, 0, 197)
        self._serial.write(frame)

        # Listen for response frames until the right one is found
        response_arrived = False
        while not rospy.is_shutdown() and not response_arrived:
            frame_received, frame = self._detect_header(self._headers)
            if frame_received:
                header = struct.unpack_from('>4B', frame)
                response_arrived = (header == (170, 0, 18, 8))
            else:
                rospy.logdebug("We got data that could not be decoded: "
                              + self._frame_to_string(frame))

        # Parse the frame to retrieve the serial number
        serial_number = self._decode(frame)
        if response_arrived and serial_number:
            return ''.join(serial_number).strip()
        else:
            return None

    def run(self):
        """
        Runs the read loop.
        """
        while not rospy.is_shutdown():
            frame_received, frame = self._detect_header(self._headers)

            if frame_received:
                data = self._decode(frame)
                if isinstance(data, OptoforceData):
                    self._publish(data)
            else:
                rospy.logdebug("We got data that could not be decoded: "
                              + self._frame_to_string(frame))


    def _detect_header(self, tree):
        """
        Read from the serial port and give back the latest data frame.

        To do so, we compare the data received with the next possible byte of
        the headers descriped in tree. This method should be called with
        self._headers. It will then recurse on subtrees of self._headers

        @param tree - dictionary structure representing the next expected bytes
        """
        try:
            raw_byte = self._serial.read()
            byte = struct.unpack('>B', raw_byte)[0]

            for header_byte, subtree in tree.items():
                if byte == header_byte:
                    if type(subtree) is int:
                        return (True, raw_byte + self._serial.read(subtree-4))
                    else:
                        success, next_bytes = self._detect_header(subtree)
                        return (success, raw_byte + next_bytes)
        except select.error as e:
            # Error code 4, meaning 'Interrupted system call'
            # It is raised when reading from the serial connexion and ROS tries
            # to stop the node.
            if e[0] != 4:
                raise

        return (False, '')

    def _decode(self, frame):
        """
        Decodes a sensor frame
        It assumes that we get an entire frame and nothing else from serial.read.

        @param frame - byte frame from the sensor
        """
        if not self._is_checksum_valid(frame):
            rospy.logwarn("Bad checksum in frame: "
                          + self._frame_to_string(frame))
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
                    try:
                        val = struct.unpack_from('>h', frame, offset)[0]
                    except struct.error as e:
                        message = ("Problem unpacking frame "
                                   + self._frame_to_string(frame)
                                   + " at offset " + str(offset) + ".")
                        rospy.logfatal(message +" Please "
                                       + "check that you set the right numbers "
                                       + "of channels and axes.")
                        rospy.signal_shutdown(message)
                        sys.exit(1)

                    # TODO Convert to Newtons (needs sensitivity report)
                    val = float(val) / self._scale[s][a]
                    force_axes.append(val)
                data.force.append(force_axes)

            return data
        elif header == (170, 0, 18, 8):
            offset = 4
            serial_number = struct.unpack_from('>8c', frame, offset)
            rospy.logdebug("We have the serial number "
                           + ''.join(serial_number))
            return serial_number
        else:
            rospy.logwarn("I can't recognize the frame's header:\n"
                          + self._frame_to_string(frame))
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

    @staticmethod
    def _frame_to_string(frame):
        """
        Build a string representing the given frame for pretty printing.

        @param frame - array of bytes (or string) represnting a frame
        @return human-readable string representation of the frame
        """
        return str(struct.unpack('>'+str(len(frame))+'B', frame))

if __name__ == '__main__':
    rospy.init_node("optoforce")
    driver = OptoforceDriver()
    driver.config()
    driver.run()
