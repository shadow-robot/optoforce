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
import logging
import array
import serial
import select # needed to handle exceptions raised by the serial module
import struct
import binascii

class OptoforceData:
    def __init__(self):
        self.status = None
        self.force = []

class OptoforceSerialNumber:
    def __init__(self, serial_number=None):
        self.serial_number = serial_number

    def __str__(self):
        return ''.join(self.serial_number).strip()

class OptoforceError(Exception):
    def __init__(self, message=""):
        self.message = str(message)

    def __str__(self):
        return self.message

    def __repr__(self):
        return "OptoforceError(" + self.message + ")"

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
    # Leaf nodes are the length of the frame associated with the header
    _headers = {170:
                   {0:
                       {18:
                           {8: 14}},
                   7:
                       {8:
                           {10: 16,
                           16: 22,
                           28: 34}}}}

    def __init__(self, port,
                sensor_type,
                scale,
                starting_index = 0):
        """
        Initialize OptoforceDriver object

        Raises serial.SerialException if there is a problem connecting to the
        serial port.

        @param port path to the device's port
        @param sensor_type string representing the sensor type (one or more
            channels, 3 or 6 axes)
        @param scale array of which the first dimension is for each sensor and
            the second for each axis of a sensor. For instance, with a
            two-channel  three-axisforce sensor:
                [[150, 75, 84], [254, 258, 275]]
        @param starting_index initial value for the topic's suffix; if not set,
            will default to 0, meaning the topic name will be "/optoforce_0"

        @raise serial.SerialException in case the device can not be found or can
            not be configured
        """
        self._logger = logging.getLogger(__name__)
        self._logger.addHandler(logging.NullHandler())

        self._serial = serial.Serial(port, 1000000, timeout=None)
        self._sensor_type = self._get_from_dict(self._daq_type_map, sensor_type,
                                          "sensor type")
        self._starting_index = starting_index

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
        self._scale = scale

        if len(self._scale) != self._nb_sensors:
            raise ValueError("Number of sensors [%i] and scaling factor vectors "
                "[%i] given doesn't match." % (self._nb_sensors, len(self._scale)))

        for x in range(self._nb_sensors):
            if len(self._scale[x]) != self._nb_axis:
                raise ValueError("Number of axis [%i] and scaling factors "
                    "[%i] given doesn't match." % (self._nb_axis, len(self._scale[x])))

    def nb_sensors(self):
        return self._nb_sensors

    def nb_axis(self):
        return self._nb_axis

    def config(self, speed = "100Hz", filter = "15Hz", zero = False):
        """
        Set the sensor's configuration.

        @ speed string representing the frequency in Hz at which data is to be
            produced. Default: "100Hz"
        @filter filtering frequency, as a string. Default: "15Hz"
        @zero boolean telling whether the sensor value should be zeroed.
        """
        speed = self._get_from_dict(self._speed_values, speed, "speed (frequency)")
        filter = self._get_from_dict(self._filter_values, filter, "filter")
        zero = self._zeroing_values[zero == True]

        config_length = 9

        header = struct.pack('>4B', 170, 0, 50, 3)
        offset = 0

        frame = array.array('B', [0] * config_length)
        struct.pack_into('>4s3B', frame, offset, header, speed, filter, zero)

        checksum = self._checksum(frame, len(frame))
        offset = len(header) + 3
        struct.pack_into('>H', frame, offset, checksum)

        self._logger.debug("Sending configuration frame "
                           + self._frame_to_string(frame))

        self._serial.write(frame)

    def request_serial_number(self):
        """
        Ask the sensor for its serial number
        """
        config_length = 6
        offset = 0

        # Build the request frame and send it
        frame = array.array('B', [0] * config_length)
        struct.pack_into('>6B', frame, offset, 171, 0, 18, 8, 0, 197)
        self._serial.write(frame)

    def read(self):
        """
        Read the next incoming frame, if any.
        """
        frame = self._detect_header(self._headers)

        return self._decode(frame)

    def _detect_header(self, tree):
        """
        Read from the serial port and give back the latest data frame.

        To do so, we compare the data received with the next possible byte of
        the headers descriped in tree. This method should be called with
        self._headers. It will then recurse on subtrees of self._headers

        @param tree - dictionary structure representing the next expected bytes

        @return the full frame, if any
        """
        try:
            raw_byte = self._serial.read()
            byte = struct.unpack('>B', raw_byte)[0]

            for header_byte, subtree in tree.items():
                if byte == header_byte:
                    if type(subtree) is int:
                        return raw_byte + self._serial.read(subtree-4)
                    else:
                        next_bytes = self._detect_header(subtree)
                        if next_bytes is not None:
                            return raw_byte + next_bytes

            self._logger.debug("I can't recognize a header in this data: "
                          + self._frame_to_string(raw_byte))
        except select.error as e:
            # Error code 4, meaning 'Interrupted system call'
            # It is raised when reading from the serial connexion and ROS tries
            # to stop the node.
            if e[0] != 4:
                raise
        except serial.serialutil.SerialException as e:
            # Same as previous except
            if str(e) != "read failed: (4, 'Interrupted system call')":
                raise

        return None

    def _decode(self, frame):
        """
        Decodes a sensor frame
        It assumes that we get an entire frame and nothing else from serial.read.

        @param frame - byte frame from the sensor
        """
        if frame is None:
            return None
        if not self._is_checksum_valid(frame):
            self._logger.error("Bad checksum in frame: "
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
                        raise OptoforceError(message +" Please "
                                       + "check that you set the right numbers "
                                       + "of channels and axes.")

                    val = float(val) / self._scale[s][a]
                    force_axes.append(val)
                data.force.append(force_axes)

            return data
        elif header == (170, 0, 18, 8):
            offset = 4
            serial_number = struct.unpack_from('>8c', frame, offset)
            self._logger.debug("The sensor has the serial number "
                               + ''.join(serial_number).strip())
            return OptoforceSerialNumber(serial_number)
        else:
            slef._logger.error("I can't recognize the header of frame: "
                          + self._frame_to_string(frame))
            return None

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

    @staticmethod
    def _get_from_dict(dictionary, key, key_name="key"):
        try:
            return dictionary[key]
        except KeyError:
            raise OptoforceError("The " + key_name + " '" + str(key) +
                "' is not part of the accepted values: " +
                ", ".join(dictionary.keys()))