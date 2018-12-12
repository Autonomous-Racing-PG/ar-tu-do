#include "car_controller.h"

CarController::CarController()
    : enabled{ true }
{
    this->drive_parameters_subscriber =
        this->node_handle.subscribe< drive_msgs::drive_param >(
            TOPIC_DRIVE_PARAM, 1, &CarController::driveParametersCallback,
            this);
    this->command_subscriber = this->node_handle.subscribe< std_msgs::String >(
        TOPIC_COMMAND, 1, &CarController::commandCallback, this);

    this->speed_pulisher =
        this->node_handle.advertise< std_msgs::Float64 >(TOPIC_FOCBOX_SPEED, 1);
    this->angle_publisher =
        this->node_handle.advertise< std_msgs::Float64 >(TOPIC_FOCBOX_ANGLE, 1);
}

void CarController::driveParametersCallback(
    const drive_msgs::drive_param::ConstPtr& parameters)
{
    this->publishDriveParameters(parameters->velocity, parameters->angle);
}

void CarController::publishDriveParameters(double raw_speed, double raw_angle)
{
    double speed = std::max(0.0, raw_speed * MAX_SPEED);
    double angle = (raw_angle * MAX_ANGLE + 1) / 2;

    if (this->enabled)
    {
        std_msgs::Float64 speed_message;
        speed_message.data = speed;
        this->speed_pulisher.publish(speed_message);
        std_msgs::Float64 angle_message;
        angle_message.data = angle;
        this->angle_publisher.publish(angle_message);
    }
    std::cout << "running: " << this->enabled << " | speed: " << speed << " | angle: " << angle << std::endl;
}

void CarController::commandCallback(
    const std_msgs::String::ConstPtr& command_message)
{
    std::string command_str = command_message->data;
    if (command_str.compare("stop") == 0)
    {
        this->enabled = false;
        this->publishDriveParameters(0, 0);
    }
    if (command_str.compare("go") == 0)
    {
        this->enabled = true;
    }
	std::cout << "command input: " << command_str << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "car_controller");
    CarController carController;
    ros::spin();
    return EXIT_SUCCESS;
}
