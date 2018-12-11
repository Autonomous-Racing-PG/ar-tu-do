#include "drive_param_converter.h"

/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "drive_param_converter");
    DriveParamConverter drive_param_coverter;

    ros::spin();

    return EXIT_SUCCESS;
}