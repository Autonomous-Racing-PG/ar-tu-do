#include "msgs_converter.h"
/**
 * @brief Starts navigatioin stack
 *
 * @param argc number of input arguments
 * @param argv input arguments
 * @return int exit code
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_stack");
    MSGSConverter msgs_converter;

    ros::spin();

    return EXIT_SUCCESS;
}