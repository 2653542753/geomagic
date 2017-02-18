/*
 * Copyright (c) 2016, Kim Lindberg Schwaner <kils@mmmi.sdu.dk>
 * Copyright (c) 2013, Dane Powell <git@danepowell.com>
 * Copyright (c) 2011, Hai Nguyen, Marc Killpack, Chi-Hung King
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "geomagic_touch/ButtonEvent.h"
#include "geomagic_touch/SetForceOutput.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"

#include "ros/ros.h"

#include <HD/hd.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/lockfree/spsc_queue.hpp>

#include <array>
#include <cmath>
#include <thread>

/*
  If this program seems to perform badly, it is probably because
  libPhantomIOLib42 seemingly SHA-2xx hashes data packages.

  TODO:
   * Check frame ID's for output poses/twists - should they match URDF model, or
     be different for geotouch_node instance launched?
   * Angular velocities of the device seems to always be zero. Is that right?
   * Test setting force/torque of the device.
   * Review joint angle offsets. Perhaps defined them in URDF description instead.
*/

enum class ButtonType { GREY, WHITE };
enum class ButtonState { RELEASED, PRESSED };

struct ButtonEvent
{
    ButtonType type;
    ButtonState event;
};

struct TouchState
{
    // Overloads 'new' so that it generates 16-bytes-aligned pointers
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Transform of the end-effector (translation in m), seen in the base frame
    // of the Touch device.
    Eigen::Affine3d transform;

    // End-effector linear+angular velocity, Cartesian space (m/s or rad/s)
    Eigen::Matrix<double, 6, 1> twist;

    // Joint angles, Joint-space (rad)
    // Waist, shoulder, elbow, yaw, pitch, roll
    std::array<double, 6> joints;
};

struct ForceCommand
{
    // Overloads 'new' so that it generates 16-bytes-aligned pointers
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Force (linear) and torque (angular) to apply on the device (N or Nm)
    Eigen::Matrix<double, 6, 1> wrench;
};

int g_calibration_style;
std::map<ButtonType, ButtonState> g_button_states;
std::map<ButtonType, ButtonState> g_button_states_prev;

boost::lockfree::spsc_queue<TouchState, boost::lockfree::capacity<1>> g_touch_states;
boost::lockfree::spsc_queue<ForceCommand, boost::lockfree::capacity<1>> g_force_commands;
boost::lockfree::spsc_queue<ButtonEvent, boost::lockfree::capacity<10>> g_button_events;

HDCallbackCode syncTouchState(void *)
{
    // Begins a frame in which the device state is guaranteed to be consistent.
    hdBeginFrame(hdGetCurrentDevice());

    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
        ROS_INFO("Updating calibration...");
        hdUpdateCalibration(g_calibration_style);
    }

    TouchState touch_state;

    // Transform (translation in mm)
    hdGetDoublev(HD_CURRENT_TRANSFORM, touch_state.transform.data());
    touch_state.transform.translation() *= 0.01; // Scale mm to meters

    // Linear and angular velocities
    hdGetDoublev(HD_CURRENT_VELOCITY, touch_state.twist.data()); // (mm/s)
    hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, touch_state.twist.data() + 3); // (rad/s)
    touch_state.twist.block<3, 1>(0, 0) *= 0.01; // Scale mm to meters

    // Joint angles
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, touch_state.joints.data()); // Waist, shoulder, elbow (rad)
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, touch_state.joints.data() + 3); // Yaw, pitch, roll (rad)

    // Buttons
    int buttons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);
    g_button_states[ButtonType::GREY] = (buttons & HD_DEVICE_BUTTON_1) ? ButtonState::PRESSED : ButtonState::RELEASED;
    g_button_states[ButtonType::WHITE] = (buttons & HD_DEVICE_BUTTON_2) ? ButtonState::PRESSED : ButtonState::RELEASED;

    // Set force feedback torque.
    // TODO: Set joint torques using HD_CURRENT_JOINT_TORQUE and
    //       HD_CURRENT_GIMBAL_TORQUE on 6DOF devices.
    // For now, just set linear force (Cartesian space).
    if (g_force_commands.read_available() > 0) {
        ForceCommand const &fcmd = g_force_commands.front();
        hdSetDoublev(HD_CURRENT_FORCE, fcmd.wrench.data());
        hdSetDoublev(HD_CURRENT_TORQUE, fcmd.wrench.data() + 3); // Deprecated
        g_force_commands.pop();
    }

    hdEndFrame(hdGetCurrentDevice());

    // Check for possible errors.
    HDErrorInfo error_info = hdGetError();

    if (error_info.errorCode != HD_SUCCESS) {
        ROS_ERROR("omniStateCallback error: %s", hdGetErrorString(error_info.errorCode));
        return HD_CALLBACK_DONE;
    }

    // Queue the (most recent) touch state.
    if (g_touch_states.write_available() == 0) {
        g_touch_states.pop();
    }

    g_touch_states.push(touch_state);

    // Queue button events.
    for (auto btype : {ButtonType::GREY, ButtonType::WHITE}) {
        if (g_button_states_prev[btype] == ButtonState::PRESSED && g_button_states[btype] == ButtonState::RELEASED) {
            // Pressed -> released transition
            g_button_events.push({btype, ButtonState::RELEASED});
        } else if (g_button_states_prev[btype] == ButtonState::RELEASED && g_button_states[btype] == ButtonState::PRESSED) {
            // Released -> pressed transition
            g_button_events.push({btype, ButtonState::PRESSED});
        }
    }

    g_button_states_prev = g_button_states;

    // Loop indefinitely
    return HD_CALLBACK_CONTINUE;
}

void onForceCommand(geometry_msgs::WrenchStamped::ConstPtr const &msg)
{
    ForceCommand fcmd;
    fcmd.wrench[0] = msg->wrench.force.x;
    fcmd.wrench[1] = msg->wrench.force.y;
    fcmd.wrench[2] = msg->wrench.force.z;
    fcmd.wrench[3] = msg->wrench.torque.x;
    fcmd.wrench[4] = msg->wrench.torque.y;
    fcmd.wrench[5] = msg->wrench.torque.z;

    // Make sure the most recent ForceCommand is queued.
    if (g_force_commands.write_available() == 0) {
        g_force_commands.pop();
    }

    g_force_commands.push(fcmd);
}

bool setForceOutput(geomagic_touch::SetForceOutputRequest &req,
                    geomagic_touch::SetForceOutputResponse &)
{
    if (req.enable_force_output) {
        hdEnable(HD_FORCE_OUTPUT);
    } else {
        hdDisable(HD_FORCE_OUTPUT);
    }

    return true;
}

sensor_msgs::JointState makeJointStateMsg(TouchState const &state)
{
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();

    msg.name.resize(6);
    msg.name[0] = "geotouch_waist";
    msg.name[1] = "geotouch_shoulder";
    msg.name[2] = "geotouch_elbow";
    msg.name[3] = "geotouch_yaw";
    msg.name[4] = "geotouch_pitch";
    msg.name[5] = "geotouch_roll";

    msg.position.resize(6);
    msg.position[0] = -state.joints[0];
    msg.position[1] = state.joints[1];
    msg.position[2] = state.joints[2] - state.joints[1];
    msg.position[3] = -state.joints[3] + M_PI;
    msg.position[4] = -state.joints[4] - 3 * M_PI / 4;
    msg.position[5] = state.joints[5] + M_PI;

    return msg;
}

geometry_msgs::PoseStamped makePoseMsg(TouchState const &state)
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "geotouch";
    msg.pose.position.x = state.transform.translation().x();
    msg.pose.position.y = state.transform.translation().y();
    msg.pose.position.z = state.transform.translation().z();
    Eigen::Quaterniond rotation(state.transform.rotation());
    msg.pose.orientation.x = rotation.x();
    msg.pose.orientation.y = rotation.y();
    msg.pose.orientation.z = rotation.z();
    msg.pose.orientation.w = rotation.w();
    return msg;
}

geometry_msgs::TwistStamped makeTwistMsg(TouchState const &state)
{
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "geotouch";
    msg.twist.linear.x = state.twist(0);
    msg.twist.linear.y = state.twist(1);
    msg.twist.linear.z = state.twist(2);
    msg.twist.angular.x = state.twist(3);
    msg.twist.angular.y = state.twist(4);
    msg.twist.angular.z = state.twist(5);
    return msg;
}

geomagic_touch::ButtonEvent makeButtonEventMsg(ButtonEvent const &event)
{
    geomagic_touch::ButtonEvent msg;

    switch (event.type) {
    case ButtonType::GREY:
        msg.button = geomagic_touch::ButtonEvent::BUTTON_GREY;
        break;
    case ButtonType::WHITE:
        msg.button = geomagic_touch::ButtonEvent::BUTTON_WHITE;
        break;
    }

    switch (event.event) {
    case ButtonState::PRESSED:
        msg.event = geomagic_touch::ButtonEvent::EVENT_PRESSED;
        break;
    case ButtonState::RELEASED:
        msg.event = geomagic_touch::ButtonEvent::EVENT_RELEASED;
        break;
    }

    return msg;
}

void calibration()
{
    // Get supported calibration style(s)
    int supported_calibration_styles;
    hdGetIntegerv(HD_CALIBRATION_STYLE, &supported_calibration_styles);

    // Choose a style
    if (supported_calibration_styles & HD_CALIBRATION_AUTO) {
        g_calibration_style = HD_CALIBRATION_AUTO;
        ROS_INFO("Calibration style: AUTO");
    } else if (supported_calibration_styles & HD_CALIBRATION_INKWELL) {
        g_calibration_style = HD_CALIBRATION_INKWELL;
        ROS_INFO("Calibration style: INKWELL");
    } else if (supported_calibration_styles & HD_CALIBRATION_ENCODER_RESET) {
        g_calibration_style = HD_CALIBRATION_ENCODER_RESET;
        ROS_INFO("Calibration style: ENCODER RESET");
    }

    // Current calibration is OK; return
    if (hdCheckCalibration() == HD_CALIBRATION_OK) {
        ROS_INFO("Current calibration OK");
        return;
    }

    // Do calibration
    ROS_INFO("Device need calibrating...");

    do {
        if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT) {
            ROS_INFO("Place the device into the inkwell for calibration...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        hdUpdateCalibration(g_calibration_style);
        HDErrorInfo error_info = hdGetError();

        if (error_info.errorCode != HD_SUCCESS) {
            ROS_ERROR("Calibration failed: %s", hdGetErrorString(error_info.errorCode));
            return;
        }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);

    ROS_INFO("Calibration complete.");
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "geotouch_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Initialize omni device
    std::string name = nh_private.param<std::string>("name", "omni1");
    HHD dev_handle = hdInitDevice(name.c_str());
    HDErrorInfo error_info = hdGetError();

    if (error_info.errorCode != HD_SUCCESS) {
        ROS_ERROR("hdInitDevice failed: %s", hdGetErrorString(error_info.errorCode));
        return EXIT_FAILURE;
    }

    ROS_INFO("Initialized device: %s.", hdGetString(HD_DEVICE_MODEL_TYPE));

    // Choose calibration method, and calibrate if necessary
    calibration();

    // Start touch scheduler in its own thread
    hdSetSchedulerRate(1000); // Should be equal to or higher than the ROS loop rate.
    HDSchedulerHandle sched_handle = hdScheduleAsynchronous(syncTouchState, nullptr, HD_MAX_SCHEDULER_PRIORITY);
    hdStartScheduler();
    error_info = hdGetError();

    if (error_info.errorCode != HD_SUCCESS) {
        ROS_ERROR("hdStartScheduler failed: %s", hdGetErrorString(error_info.errorCode));
        hdDisableDevice(dev_handle);
        return EXIT_FAILURE;
    }

    // Process ROS events in this thread
    ros::Publisher pub_joint_states = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher pub_poses = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    ros::Publisher pub_twists = nh.advertise<geometry_msgs::TwistStamped>("twist", 1);
    ros::Publisher pub_button_events = nh.advertise<geomagic_touch::ButtonEvent>("button_event", 10);
    ros::Subscriber sub_force_cmds = nh.subscribe("force_command", 1, onForceCommand);
    ros::ServiceServer srv_force_output = nh.advertiseService("set_force_output", setForceOutput);
    ros::Rate loop_rate(500);

    while (nh.ok()) {
        loop_rate.sleep();

        if (g_touch_states.read_available() > 0) {
            TouchState const &touch_state = g_touch_states.front();
            pub_joint_states.publish(makeJointStateMsg(touch_state));
            pub_poses.publish(makePoseMsg(touch_state));
            pub_twists.publish(makeTwistMsg(touch_state));
            g_touch_states.pop();
        }

        while (g_button_events.read_available() > 0) {
            ButtonEvent const &btn_event = g_button_events.front();
            pub_button_events.publish(makeButtonEventMsg(btn_event));
            g_button_events.pop();
        }

        ros::spinOnce(); // Process waiting callbacks
    }

    // Clean up
    hdStopScheduler();
    hdUnschedule(sched_handle);
    hdDisableDevice(dev_handle);

    return EXIT_SUCCESS;
}
