#include "drive_param_converter.h"

/**
 * @brief Starts drive parameter converter for gazebo simulation
 *
 * @param argc number of input arguments
 * @param argv input arguments
 * @return int exit code
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "drive_param_converter");
    DriveParamConverter drive_param_coverter;

    ros::spin();

    return EXIT_SUCCESS;
}