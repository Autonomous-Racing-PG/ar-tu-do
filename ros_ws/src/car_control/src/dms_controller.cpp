#include "dms_controller.h"

DMSController::DMSController()
    : last_dms_message_received{ 0 }
    , running{ false }
{
    this->dms_subscriber = this->node_handle.subscribe< std_msgs::Int64 >(
        TOPIC_DMS, 1, &DMSController::dmsCallback, this);

    this->command_pulisher =
        this->node_handle.advertise< std_msgs::String >(TOPIC_COMMAND, 1);
}

void DMSController::checkDMS()
{
    struct timeval time_struct;
    gettimeofday(&time_struct, NULL);
    long int timestamp = time_struct.tv_sec * 1000 + time_struct.tv_usec / 1000;

    //
    if (running)
    {
        // last dms message is outdated
        if (last_dms_message_received + DMS_EXPIRATION < timestamp)
        {

            running = false;
            std_msgs::String command_message;
            command_message.data = "stop";
            this->command_pulisher.publish(command_message);
        }
        // dms works fine
        else
        {
        }
    }
    if (!running)
    {
        // dms is up-to-date
        if (last_dms_message_received + DMS_EXPIRATION >= timestamp)
        {

            running = true;
            std_msgs::String command_message;
            command_message.data = "go";
            this->command_pulisher.publish(command_message);
        }
        // dms still not working
        else
        {
        }
    }
}

void DMSController::dmsCallback(const std_msgs::Int64::ConstPtr& dms_message)
{
    this->last_dms_message_received = dms_message->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dms_controller");
    DMSController dmsController;

    ros::Rate loop_rate(DMS_CHECK_RATE);
    while (ros::ok())
    {
        dmsController.checkDMS();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return EXIT_SUCCESS;
}
