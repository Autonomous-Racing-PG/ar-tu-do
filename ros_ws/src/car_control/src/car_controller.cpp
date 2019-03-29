#include "car_controller.h"
#include "car_config.h"

#include <boost/algorithm/clamp.hpp>

CarController::CarController()
    : m_motor_unlocked{ false }
{
    this->m_drive_parameters_subscriber =
        this->m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_DRIVE_PARAM, 1,
                                                               &CarController::driveParametersCallback, this);
    this->m_drive_mode_subscriber =
        this->m_node_handle.subscribe<std_msgs::Int32>(TOPIC_DRIVE_MODE, 1, &CarController::driveModeCallback, this);

    this->m_speed_pulisher = this->m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_SPEED, 1);
    this->m_angle_publisher = this->m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_ANGLE, 1);
    this->m_brake_publisher = this->m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_BRAKE, 1);
}

void CarController::driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    if (this->m_motor_unlocked)
    {
        this->publishDriveParameters(parameters->velocity, parameters->angle);
    }
}

void CarController::publishDriveParameters(double relative_speed, double relative_angle)
{
    double speed = relative_speed * car_config::MAX_RPM_ELECTRICAL;
    double angle = (relative_angle * car_config::MAX_SERVO_POSITION + car_config::MAX_SERVO_POSITION) / 2;

    std_msgs::Float64 speed_message;
    speed_message.data = speed;
    this->m_speed_pulisher.publish(speed_message);

    std_msgs::Float64 angle_message;
    angle_message.data = angle;
    this->m_angle_publisher.publish(angle_message);

    ROS_DEBUG_STREAM("running: "
                     << " | speed: " << speed << " | angle: " << angle);
}

void CarController::driveModeCallback(const std_msgs::Int32::ConstPtr& drive_mode_message)
{
    DriveMode mode = (DriveMode)drive_mode_message->data;
    ROS_ASSERT_MSG(mode == DriveMode::LOCKED || mode == DriveMode::MANUAL || mode == DriveMode::AUTONOMOUS,
                   "Unknown drive mode.");
    if (this->m_motor_unlocked && mode == DriveMode::LOCKED)
    {
        this->stop();
    }
    this->m_motor_unlocked = mode != DriveMode::LOCKED;
}

void CarController::stop()
{
    this->publishDriveParameters(0, 0);

    std_msgs::Float64 brake_message;
    brake_message.data = 0;
    this->m_brake_publisher.publish(brake_message);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "car_controller");
    CarController carController;
    ros::spin();
    return EXIT_SUCCESS;
}
