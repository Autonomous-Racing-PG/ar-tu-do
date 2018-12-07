#include "car_control.h"

CarController::CarController()
    : run{ true }
{
    this->driveParametersSubscriber = this->nodeHandle.subscribe< drive_msgs::drive_param >(
        TOPIC_DRIVE_PARAM, 1, &CarController::driveParametersCallback, this);    
    this->commandSubscriber = this->nodeHandle.subscribe< std_msgs::String >(TOPIC_COMMAND, 1,
                                               &CarController::command_callback, this);

    this->speedPublisher = this->nodeHandle.advertise< std_msgs::Float64 >(TOPIC_FOCBOX_SPEED, 1);
    this->anglePublisher = this->nodeHandle.advertise< std_msgs::Float64 >(TOPIC_FOCBOX_ANGLE, 1);
}

void CarController::driveParametersCallback(
    const drive_msgs::drive_param::ConstPtr& parameters)
{
    this->publishDriveParameters(parameters->velocity, parameters->angle);
}

void CarController::publishDriveParameters(double rawSpeed, double rawAngle)
{
    double speed = std::max(0.0, rawSpeed * MAX_SPEED);
    double angle = (rawAngle * MAX_ANGLE + 1) / 2;

    if (this->run)
    {
        std_msgs::Float64 speedMessage;
        speedMessage.data = speed;
        this->speedPublisher.publish(speedMessage);
        std_msgs::Float64 angleMessage;
        angleMessage.data = angle;
        this->anglePublisher.publish(angleMessage);
    } else {
		std::cout << "not running - ";
	}
    std::cout << "speed: " << speed << " | angle: " << angle << std::endl;
}

void CarController::command_callback(const std_msgs::String::ConstPtr& command)
{
    std::string command_str = command->data;
    if (command_str.compare("stop") == 0)
    {
        this->run = false;
        this->publishDriveParameters(0, 0);
    }
    if (command_str.compare("go") == 0)
    {
        this->run = true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "car_controller");
    CarController carController;
    ros::spin();
    return 0;
}
