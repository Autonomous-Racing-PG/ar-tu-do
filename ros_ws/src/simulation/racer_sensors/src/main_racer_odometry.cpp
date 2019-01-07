#include "racer_odometry.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "racer_odometry");
    RacerOdometry racerOdometry;
    ros::spin();
    return EXIT_SUCCESS;
}
