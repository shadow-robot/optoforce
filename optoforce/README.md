# optoforce

ROS driver for the [Optoforce sensor](http://optoforce.com/3dsensor/).

## Optoforce models supported

- Single-channel 3 axis force sensor
- Multi-channel 3 axis force sensor (4 channels)
- Single-channel 6 axis force sensor

## Driver description

The driver relies on the pySerial library to get data from the sensors through USB.

It publishes a geometry_msgs/WrenchStamped for every sensor.


