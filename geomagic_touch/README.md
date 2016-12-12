# geomagic_touch

ROS package for interfacing Geomagic haptic devices with ROS.

Parameters:
- ~omni_name (default: omni1)

Publishes:
- joint_states (sensor_msgs/JointState): The joint angles of the device.
- touch_state (phantom_omni/TouchState): The pose, twist and button state of the device.

Subscribes:
- force_command (geometry_msgs/WrenchStamped): Force feedback to be displayed on the omni.

This is based on the [original phantom_omni package](http://www.ros.org/wiki/phantom_omni).
