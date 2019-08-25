#include "car_controller.h"
#include "car_config.h"

#include <boost/algorithm/clamp.hpp>

CarController::CarController()
    : m_drive_param_lock{ true }
    , m_emergency_stop_lock{ true }
{
    this->m_drive_parameters_subscriber =
        this->m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_DRIVE_PARAM, 1,
                                                               &CarController::driveParametersCallback, this);
    this->m_drive_mode_subscriber =
        this->m_node_handle.subscribe<std_msgs::Int32>(TOPIC_DRIVE_MODE, 1, &CarController::driveModeCallback, this);
    this->m_emergency_stop_subscriber =
        this->m_node_handle.subscribe<std_msgs::Bool>(TOPIC_EMERGENCY_STOP, 1, &CarController::emergencyStopCallback,
                                                      this);

    this->m_speed_publisher = this->m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_SPEED, 1);
    this->m_angle_publisher = this->m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_ANGLE, 1);
    this->m_brake_publisher = this->m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_BRAKE, 1);
}

void CarController::driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    this->publishDriveParameters((m_drive_param_lock || m_emergency_stop_lock) ? 0 : parameters->velocity,
                                 m_drive_param_lock ? 0 : parameters->angle);
}

void CarController::publishDriveParameters(double relative_speed, double relative_angle)
{
    double speed = relative_speed * car_config::MAX_RPM_ELECTRICAL;
    double angle = (relative_angle * car_config::MAX_SERVO_POSITION + car_config::MAX_SERVO_POSITION) / 2;

    this->publishSpeed(speed);
    this->publishAngle(angle);

    ROS_DEBUG_STREAM("running: "
                     << " | speed: " << speed << " | angle: " << angle);
}

void CarController::publishSpeed(double speed)
{
    std_msgs::Float64 speed_message;
    speed_message.data = speed;
    this->m_speed_publisher.publish(speed_message);
}

void CarController::publishAngle(double angle)
{
    std_msgs::Float64 angle_message;
    angle_message.data = angle;
    this->m_angle_publisher.publish(angle_message);
}

void CarController::driveModeCallback(const std_msgs::Int32::ConstPtr& drive_mode_message)
{
    this->m_current_drive_mode = (DriveMode)drive_mode_message->data;
    this->m_drive_param_lock = this->m_current_drive_mode == DriveMode::LOCKED;
    if (this->m_drive_param_lock)
        this->stop();
}

void CarController::emergencyStopCallback(const std_msgs::Bool::ConstPtr& emergency_stop_message)
{
    bool enable_emergency_stop = emergency_stop_message->data && this->m_current_drive_mode != DriveMode::MANUAL;
    this->m_emergency_stop_lock = enable_emergency_stop;
    if (this->m_emergency_stop_lock)
        this->stop();
}

void CarController::stop()
{
    this->publishSpeed(0);

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
