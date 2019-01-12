#include "car_controller.h"
#include "car_config.h"

#include "car_control.h"

#include "car_control.h"

#include "car_control.h"

#include "car_control.h"

CarController::CarController()
    : enabled{ false }
{
    this->drive_parameters_subscriber =
        this->node_handle.subscribe<drive_msgs::drive_param>(TOPIC_DRIVE_PARAM, 1,
                                                             &CarController::driveParametersCallback, this);
    this->command_subscriber =
        this->node_handle.subscribe<std_msgs::String>(TOPIC_COMMAND, 1, &CarController::commandCallback, this);

    this->speed_pulisher = this->node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_SPEED, 1);
    this->angle_publisher = this->node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_ANGLE, 1);
    this->break_publisher = this->node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_BREAK, 1);
}

void CarController::driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    this->publishDriveParameters(parameters->velocity, parameters->angle);
}

void CarController::publishDriveParameters(double raw_speed, double raw_angle)
{
    double speed = raw_speed * car_config::MAX_RPM_ELECTRICAL;
    double angle = (raw_angle * car_config::MAX_SERVO_POSITION + car_config::MAX_SERVO_POSITION) / 2;

    if (this->enabled)
    {
        std_msgs::Float64 speed_message;
        speed_message.data = speed;
        this->speed_pulisher.publish(speed_message);
        std_msgs::Float64 angle_message;
        angle_message.data = angle;
        this->angle_publisher.publish(angle_message);
    }
    ROS_DEBUG_STREAM("running: " << this->enabled << " | speed: " << speed << " | angle: " << angle << std::endl);
}

void CarController::commandCallback(const std_msgs::String::ConstPtr& command_message)
{
    std::string command_str = command_message->data;
    if (command_str.compare(COMMAND_STOP) == 0)
    {
        this->enabled = false;
        this->publishDriveParameters(0, 0);
        // break
        std_msgs::Float64 break_message;
        break_message.data = 0;
        this->break_publisher.publish(break_message);
    }
    if (command_str.compare(COMMAND_GO) == 0)
    {
        this->enabled = true;
    }
    ROS_INFO_STREAM("command input: " << command_str << std::endl);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "car_controller");
    CarController carController;
    ros::spin();
    return EXIT_SUCCESS;
}
