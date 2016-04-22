# optoforce

ROS driver for the [Optoforce sensor](http://optoforce.com/3dsensor/).

## Optoforce models supported

- Single-channel 3 axis force sensor
- Multi-channel 3 axis force sensor (4 channels)
- Single-channel 6 axis force sensor

## Driver description

The driver relies on the pySerial library to get data from the sensors through USB.

It publishes a geometry_msgs/WrenchStamped for every sensor.


## Quickstart

Have a look at `optoforce.launch` for the node's parameters and their values.

It also loads parameters from the `multi_channel_3_axis_generic_scale.yaml` file. They are ratios from raw data to Newton, as found in the sensor's sensitivity report. If you have a single channel force sensor, have a look at `single_channel_3_axis_generic_scale.yaml`.

### Shadow robot's hand

- `optoforce_hand.launch` start the optoforce node configured for the hand
- `rviz.launch` will start Rviz configured to display the hand