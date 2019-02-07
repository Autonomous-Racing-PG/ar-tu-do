#include "car_controller.h"
#include "car_config.h"
#include "car_control.h"

CarController::CarController()
    : m_enabled{ false }
{
    this->m_drive_parameters_subscriber =
        this->m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_DRIVE_PARAM, 1,
                                                               &CarController::driveParametersCallback, this);
    this->m_command_subscriber =
        this->m_node_handle.subscribe<std_msgs::String>(TOPIC_COMMAND, 1, &CarController::commandCallback, this);

    this->m_speed_pulisher = this->m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_SPEED, 1);
    this->m_angle_publisher = this->m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_ANGLE, 1);
    this->m_brake_publisher = this->m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_BRAKE, 1);
}

void CarController::driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    this->publishDriveParameters(parameters->velocity, parameters->angle);
}

void CarController::publishDriveParameters(double raw_speed, double raw_angle)
{
    double speed = raw_speed * car_config::MAX_RPM_ELECTRICAL;
    double angle = (raw_angle * car_config::MAX_SERVO_POSITION + car_config::MAX_SERVO_POSITION) / 2;

    if (this->m_enabled)
    {
        std_msgs::Float64 speed_message;
        speed_message.data = speed;
        this->m_speed_pulisher.publish(speed_message);
        std_msgs::Float64 angle_message;
        angle_message.data = angle;
        this->m_angle_publisher.publish(angle_message);
    }
    ROS_DEBUG_STREAM("running: " << this->m_enabled << " | speed: " << speed << " | angle: " << angle);
}

void CarController::commandCallback(const std_msgs::String::ConstPtr& command_message)
{
    std::string command_str = command_message->data;
    if (command_str.compare(COMMAND_STOP) == 0)
    {
        this->publishDriveParameters(0, 0);
        this->m_enabled = false;
        // brake
        std_msgs::Float64 brake_message;
        brake_message.data = 0;
        this->m_brake_publisher.publish(brake_message);
    }
    if (command_str.compare(COMMAND_GO) == 0)
    {
        this->m_enabled = true;
    }
    ROS_INFO_STREAM("command input: " << command_str);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "car_controller");
    CarController carController;
    ros::spin();
    return EXIT_SUCCESS;
}
