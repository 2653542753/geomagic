#include "geomagic_touch/PhantomButtonEvent.h"
#include "geomagic_touch/OmniFeedback.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

#include <cmath>
#include <array>
#include <thread>
#include <mutex>

class OmniState
{
public:
    enum class ButtonState
    {
        RELEASED, PRESSED
    };

    OmniState()
    {
        buttons.fill(ButtonState::RELEASED);
    }

    hduVector3Dd position; // mm
    hduVector3Dd velocity; // mm/s
    hduMatrix transform; // transform of the end-effector TODO with reference to??
    hduVector3Dd angular_velocity; // rad/s
    hduVector3Dd joint_angles; // rad
    hduVector3Dd gimbal_angles; // rad
    std::array<ButtonState, 2> buttons;
    std::mutex mutex;
};

// Updates the local omni state object
HDCallbackCode omniStateCallback(void *userdata)
{
    OmniState *omni_state = static_cast<OmniState *>(userdata);

    std::lock_guard<std::mutex> lk(omni_state->mutex);

    hdBeginFrame(hdGetCurrentDevice());
    hdGetDoublev(HD_CURRENT_POSITION, omni_state->position);
    hdGetDoublev(HD_CURRENT_VELOCITY, omni_state->velocity);
    hdGetDoublev(HD_CURRENT_TRANSFORM, omni_state->transform);
    hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, omni_state->angular_velocity);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joint_angles);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, omni_state->gimbal_angles);
    int buttons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);
    hdEndFrame(hdGetCurrentDevice());

    omni_state->buttons[0] = (buttons & HD_DEVICE_BUTTON_1) ? OmniState::ButtonState::PRESSED : OmniState::ButtonState::RELEASED;
    omni_state->buttons[1] = (buttons & HD_DEVICE_BUTTON_2) ? OmniState::ButtonState::PRESSED : OmniState::ButtonState::RELEASED;

    HDErrorInfo error_info;

    if (HD_DEVICE_ERROR(error_info = hdGetError())) {
        ROS_ERROR("Updating omni state failed: %s", hdGetErrorString(error_info.errorCode));
        return HD_CALLBACK_DONE;
    }

    // Loop indefinitely
    return HD_CALLBACK_CONTINUE;
}

sensor_msgs::JointState makeJointStateMsg(OmniState &omni_state)
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

    std::lock_guard<std::mutex> lk(omni_state.mutex);

    msg.position[0] = -omni_state.joint_angles[0];
    msg.position[1] = omni_state.joint_angles[1];
    msg.position[2] = omni_state.joint_angles[2] - omni_state.joint_angles[1];
    msg.position[3] = -omni_state.gimbal_angles[0] + M_PI;
    msg.position[4] = -omni_state.gimbal_angles[1] - 3 * M_PI / 4;
    msg.position[5] = omni_state.gimbal_angles[2] + M_PI;

    return msg;
}

geomagic_touch::PhantomButtonEvent makeButtonEventMsg(OmniState &omni_state)
{
    geomagic_touch::PhantomButtonEvent msg;

    std::lock_guard<std::mutex> lk(omni_state.mutex);
    msg.grey_button = (omni_state.buttons[0] == OmniState::ButtonState::PRESSED) ? 1 : 0;
    msg.white_button = (omni_state.buttons[1] == OmniState::ButtonState::PRESSED) ? 1 : 0;

    return msg;
}

// Automatic Calibration of Phantom Device - No character inputs
void calibration()
{
    if (hdCheckCalibration() == HD_CALIBRATION_OK) {
        return;
    }

    ROS_INFO("Device need calibrating...");

    // Get supported calibration style(s)
    int supported_calibration_styles;
    hdGetIntegerv(HD_CALIBRATION_STYLE, &supported_calibration_styles);
    int calibration_style;

    if (supported_calibration_styles & HD_CALIBRATION_ENCODER_RESET) {
        calibration_style = HD_CALIBRATION_ENCODER_RESET;
        ROS_INFO("HD_CALIBRATION_ENCODER_RESET");
    }

    if (supported_calibration_styles & HD_CALIBRATION_INKWELL) {
        calibration_style = HD_CALIBRATION_INKWELL;
        ROS_INFO("HD_CALIBRATION_INKWELL");
    }

    if (supported_calibration_styles & HD_CALIBRATION_AUTO) {
        calibration_style = HD_CALIBRATION_AUTO;
        ROS_INFO("HD_CALIBRATION_AUTO");
    }

    // Do calibration
    do {
        if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT) {
            ROS_INFO("Please place the device into the inkwell for calibration.");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        hdUpdateCalibration(calibration_style);
        HDErrorInfo error_info;

        if (HD_DEVICE_ERROR(error_info = hdGetError())) {
            ROS_ERROR("Calibration failed: %s", hdGetErrorString(error_info.errorCode));
            return;
        }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);

    ROS_INFO("Calibration OK");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omni_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string name = nh_private.param<std::string>("omni_name", "omni1");

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

    HDErrorInfo error_info;
    HHD hDev = hdInitDevice(name.c_str());

    if (HD_DEVICE_ERROR(error_info = hdGetError())) {
        //hduPrintError(stderr, &error, "Failed to initialize haptic device");
        ROS_ERROR("hdInitDevice failed: %s", hdGetErrorString(error_info.errorCode));
        return EXIT_FAILURE;
    }

    ROS_INFO("Initialized device: %s.", hdGetString(HD_DEVICE_MODEL_TYPE));

    hdEnable(HD_FORCE_OUTPUT); // Enable force output (turn on all motors)

    if (HD_DEVICE_ERROR(error_info = hdGetError())) {
        ROS_ERROR("hdStartScheduler failed: %s", hdGetErrorString(error_info.errorCode));
        hdDisableDevice(hDev);
        return EXIT_FAILURE;
    }

    calibration();

    hdSetSchedulerRate(1000); // TODO

    OmniState omni_state;
    HDSchedulerHandle hSch = hdScheduleAsynchronous(omniStateCallback, &omni_state, HD_MAX_SCHEDULER_PRIORITY);
    hdStartScheduler();

    ros::Rate loop_rate(100); // TODO

    while (nh.ok()) {
        loop_rate.sleep();
        ros::spinOnce(); // Process waiting callbacks

        // Publish current omni state
        joint_pub.publish(makeJointStateMsg(omni_state));
    }

    // Clean up
    hdStopScheduler();
    hdUnschedule(hSch);
    hdDisableDevice(hDev);

    return EXIT_SUCCESS;
}
