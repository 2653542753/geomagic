# geomagic_touch

ROS package for interfacing Geomagic haptic devices with ROS.

Parameters:
- ~name (default: omni1): The name of the device (given by Geomagic Touch driver setup).

Publishes:
- joint_states (sensor_msgs/JointState): The joint angles of the device.
- pose (geomagic_touch/TouchState): The pose of the device.
- twist (geomagic_touch/TouchState): The twist of the device.
- button_event (geomagic_touch/TouchState): Buttons events (pressed, released).

Subscribes:
- force_command (geometry_msgs/WrenchStamped): Force feedback to be displayed on the device.

This is based on the [original phantom_omni package](http://www.ros.org/wiki/phantom_omni).

Haptic device drivers and the OpenHaptics toolkit may be downloaded from
http://developer.geomagic.com/ (requires registration).
