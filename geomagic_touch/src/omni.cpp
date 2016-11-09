#include "geomagic_touch/TouchState.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "tf2_eigen/tf2_eigen.h"
#include "ros/ros.h"

#include <HD/hd.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <mutex>
#include <thread>

struct TouchState
{
    enum class ButtonState
    {
        RELEASED, PRESSED
    };

    TouchState()
    {
        transform.setIdentity();
        twist.setZero();
        joints.fill(0);
        buttons.fill(ButtonState::RELEASED);
    }

    // Overloads 'new' so that it generates 16-bytes-aligned pointers
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Transform of the end-effector (translation in m)
    // TODO with reference to where??
    Eigen::Affine3d transform;

    // End-effector linear+angular velocity, Cartesian space (m/s or rad/s)
    Eigen::Matrix<double, 6, 1> twist;

    // Joint angles, Joint-space (rad)
    // Waist, shoulder, elbow, yaw, pitch, roll
    std::array<double, 6> joints;

    // Button states
    std::array<ButtonState, 2> buttons;

    std::mutex mutex;
};

struct ForceCommand
{
    ForceCommand()
    {
        wrench.setZero();
    }

    // Overloads 'new' so that it generates 16-bytes-aligned pointers
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Force (linear) and torque (angular) to apply on the device (N or Nm)
    Eigen::Matrix<double, 6, 1> wrench;

    std::mutex mutex;
};

int g_calibration_style;
TouchState g_omni_state;
ForceCommand g_omni_cmd;

// Updates the local omni state object
HDCallbackCode omniStateCallback(void *)
{
    // Begins a frame in which the device state is guaranteed to be consistent
    std::lock_guard<std::mutex> lk_state(g_omni_state.mutex);
    std::lock_guard<std::mutex> lk_cmd(g_omni_cmd.mutex);
    hdBeginFrame(hdGetCurrentDevice());

    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
        ROS_INFO("Updating calibration...");
        hdUpdateCalibration(g_calibration_style);
    }

    // Get most current device state
    hdGetDoublev(HD_CURRENT_TRANSFORM, g_omni_state.transform.data()); // (translation in mm)
    hdGetDoublev(HD_CURRENT_VELOCITY, g_omni_state.twist.data()); // (mm/s)
    hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, g_omni_state.twist.data() + 3); // (rad/s)
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, g_omni_state.joints.data()); // Waist, shoulder, elbow (rad)
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, g_omni_state.joints.data() + 3); // Yaw, pitch, roll (rad)
    int buttons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);

    // TODO set joint torques using HD_CURRENT_JOINT_TORQUE and
    //      HD_CURRENT_GIMBAL_TORQUE on 6DOF devices.
    // For now, just set linear force (Cartesian space).
    hdSetDoublev(HD_CURRENT_FORCE, g_omni_cmd.wrench.data());
    hdSetDoublev(HD_CURRENT_TORQUE, g_omni_cmd.wrench.data() + 3); // Deprecated
    hdEndFrame(hdGetCurrentDevice());

    g_omni_state.buttons[0] = (buttons & HD_DEVICE_BUTTON_1) ? TouchState::ButtonState::PRESSED : TouchState::ButtonState::RELEASED;
    g_omni_state.buttons[1] = (buttons & HD_DEVICE_BUTTON_2) ? TouchState::ButtonState::PRESSED : TouchState::ButtonState::RELEASED;

    // Scale mm to meters
    g_omni_state.transform.translation() *= 0.01;
    g_omni_state.twist.block<3, 1>(0, 0) *= 0.01;

    HDErrorInfo error_info;

    if (HD_DEVICE_ERROR(error_info = hdGetError())) {
        ROS_ERROR("omniStateCallback error: %s", hdGetErrorString(error_info.errorCode));
        return HD_CALLBACK_DONE;
    }

    // Loop indefinitely
    return HD_CALLBACK_CONTINUE;
}

// Sets force command
void setForceCmd(geometry_msgs::WrenchStamped::ConstPtr const &msg)
{
    std::lock_guard<std::mutex> lk(g_omni_cmd.mutex);
    g_omni_cmd.wrench[0] = msg->wrench.force.x;
    g_omni_cmd.wrench[1] = msg->wrench.force.y;
    g_omni_cmd.wrench[2] = msg->wrench.force.z;
    g_omni_cmd.wrench[3] = msg->wrench.torque.x;
    g_omni_cmd.wrench[4] = msg->wrench.torque.y;
    g_omni_cmd.wrench[5] = msg->wrench.torque.z;
}

sensor_msgs::JointState makeJointStateMsg(TouchState &state)
{
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.name.resize(6);
    msg.position.resize(6);

    msg.name[0] = "waist";
    msg.name[1] = "shoulder";
    msg.name[2] = "elbow";
    msg.name[3] = "yaw";
    msg.name[4] = "pitch";
    msg.name[5] = "roll";

    std::lock_guard<std::mutex> lk(state.mutex);

    // TODO review these offsets
    msg.position[0] = -state.joints[0];
    msg.position[1] = state.joints[1];
    msg.position[2] = state.joints[2] - state.joints[1];
    msg.position[3] = -state.joints[3] + M_PI;
    msg.position[4] = -state.joints[4] - 3 * M_PI / 4;
    msg.position[5] = state.joints[5] + M_PI;

    return msg;
}

geomagic_touch::TouchState makeTouchStateMsg(TouchState &state)
{
    geomagic_touch::TouchState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "geomagic_touch_base";
    msg.buttons.resize(2);

    std::lock_guard<std::mutex> lk(state.mutex);
    msg.buttons[0] = (state.buttons[0] == TouchState::ButtonState::PRESSED) ? 1 : 0;
    msg.buttons[1] = (state.buttons[1] == TouchState::ButtonState::PRESSED) ? 1 : 0;

    msg.pose.position.x = state.transform.translation().x();
    msg.pose.position.y = state.transform.translation().y();
    msg.pose.position.z = state.transform.translation().z();

    Eigen::Quaterniond rot(state.transform.rotation());
    msg.pose.orientation.x = rot.x();
    msg.pose.orientation.y = rot.y();
    msg.pose.orientation.z = rot.z();
    msg.pose.orientation.w = rot.w();

    return msg;
}

// FIXME
// Frame increments in "ITP" frame for the Raven
void cheat_itp(Eigen::Affine3d const &tf, Eigen::Quaterniond &rot_incr, Eigen::Vector3d &pos_incr)
{
    static Eigen::Vector3d pos_prev(0, 0, 0);
    static Eigen::Quaterniond rot_prev = Eigen::Quaterniond::Identity();

    // Map into ITP frame
    Eigen::Affine3d omni_to_ITP;
    omni_to_ITP.matrix() << 0, 1, 0, 0,
                            0, 0, -1, 0,
                            -1, 0, 0, 0,
                            0, 0, 0, 1;
    Eigen::Affine3d tf_itp = tf * omni_to_ITP;

    // Position increment
    Eigen::Vector3d pos_curr(tf_itp.translation());
    pos_incr = pos_prev - pos_curr;

    // Rotation increment ( TODO slerp? )
    Eigen::Quaterniond rot_curr(tf_itp.rotation());
    rot_incr = rot_prev.inverse() * rot_curr;

    pos_prev = pos_curr;
    rot_prev = rot_curr;
}

geomagic_touch::TouchState makeEndEffectorRelativeMsg(TouchState &state)
{
    geomagic_touch::TouchState msg;
    msg.header.stamp = ros::Time::now();
    msg.buttons.resize(2);

    std::lock_guard<std::mutex> lk(state.mutex);
    msg.buttons[0] = (state.buttons[0] == TouchState::ButtonState::PRESSED) ? 1 : 0;
    msg.buttons[1] = (state.buttons[1] == TouchState::ButtonState::PRESSED) ? 1 : 0;

    Eigen::Quaterniond rot_incr;
    Eigen::Vector3d pos_incr;
    cheat_itp(state.transform, rot_incr, pos_incr);

    msg.pose.position.x = pos_incr.x();
    msg.pose.position.y = pos_incr.y();
    msg.pose.position.z = pos_incr.z();
    msg.pose.orientation.x = rot_incr.x();
    msg.pose.orientation.y = rot_incr.y();
    msg.pose.orientation.z = rot_incr.z();
    msg.pose.orientation.w = rot_incr.w();

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
        HDErrorInfo error_info;

        if (HD_DEVICE_ERROR(error_info = hdGetError())) {
            ROS_ERROR("Calibration failed: %s", hdGetErrorString(error_info.errorCode));
            return;
        }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);

    ROS_INFO("Calibration complete.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omni_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // std::string name = "GeoTouchLeft";
    std::string name = nh_private.param<std::string>("omni_name", "omni1");

    ros::Publisher pub_joint_state = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher pub_state = nh.advertise<geomagic_touch::TouchState>("touch_state", 1);
    ros::Publisher pub_relative_incr = nh.advertise<geomagic_touch::TouchState>("increment_itp", 1);
    ros::Subscriber sub_force_cmd = nh.subscribe("force_command", 1, setForceCmd);

    HDErrorInfo error_info;
    HHD hDev = hdInitDevice(name.c_str());

    if (HD_DEVICE_ERROR(error_info = hdGetError())) {
        //hduPrintError(stderr, &error, "Failed to initialize haptic device");
        ROS_ERROR("hdInitDevice failed: %s", hdGetErrorString(error_info.errorCode));
        return EXIT_FAILURE;
    }

    ROS_INFO("Initialized device: %s.", hdGetString(HD_DEVICE_MODEL_TYPE));

    calibration(); // Choose calibration method, and calibrate if necessary

    hdEnable(HD_FORCE_OUTPUT); // Enable force output (turn on all motors)
    hdSetSchedulerRate(1000); // TODO
    HDSchedulerHandle hSch = hdScheduleAsynchronous(omniStateCallback, nullptr, HD_DEFAULT_SCHEDULER_PRIORITY);
    hdStartScheduler();

    if (HD_DEVICE_ERROR(error_info = hdGetError())) {
        ROS_ERROR("hdStartScheduler failed: %s", hdGetErrorString(error_info.errorCode));
        hdDisableDevice(hDev);
        return EXIT_FAILURE;
    }

    ros::Rate loop_rate(500); // TODO

    while (nh.ok()) {
        loop_rate.sleep();

        // Publish current omni state
        pub_joint_state.publish(makeJointStateMsg(g_omni_state));
        pub_state.publish(makeTouchStateMsg(g_omni_state));
        pub_relative_incr.publish(makeEndEffectorRelativeMsg(g_omni_state));

        ros::spinOnce(); // Process waiting callbacks
    }

    // Clean up
    hdStopScheduler();
    hdUnschedule(hSch);
    hdDisableDevice(hDev);

    return EXIT_SUCCESS;
}
