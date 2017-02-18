# geomagic_touch

ROS package for interfacing Geomagic haptic devices with ROS.

Parameters:
- ~name (default: omni1): The name of the device (given by Geomagic Touch driver
  setup).

Publishes:
- joint_states (sensor_msgs/JointState): The joint angles of the device.
- pose (geometry_msgs/PoseStamped): The pose of the device.
- twist (geometry_msgs/TwistStamped): The twist of the device.
- button_event (geomagic_touch/ButtonEvent): Buttons events (pressed, released).

Subscribes:
- force_command (geometry_msgs/WrenchStamped): Force feedback to be displayed on
  the device.

Services:
- set_force_output (geomagic_touch/SetForceOutput): Enable or disable the force
  output of the device.

This is based on the [original phantom_omni package](http://www.ros.org/wiki/phantom_omni).

Haptic device drivers and the OpenHaptics toolkit may be downloaded from
http://developer.geomagic.com/ (requires registration).
