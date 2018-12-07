#include "dms_controller.h"

DMSController::DMSController()
{
	out_cmd_ = nh_.advertise< std_msgs::String >(TOPIC_COMMAND, 1);
}

void DMSController::keyLoop()
{
	int c = getch();
    while (ros::ok())
    {

        if (c == KEYCODE_SPACE)
        {
			std::cout << "pressing: " << c << std::endl;
        } else {
			std::cout << "not pressing" << std::endl;
		}
    }
}
	
int DMSController::getch()
{
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt); // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);               // disable buffering
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

    int c = getchar(); // read character (non-blocking)

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
    return c;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dms_control");
    DMSController dms_controller;
	dms_controller.keyLoop();
    return 0;
}
