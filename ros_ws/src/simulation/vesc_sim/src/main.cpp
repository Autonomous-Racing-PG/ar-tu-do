#include "vesc_sim_driver.h"

/**
 * @brief Starts vesc simulator for gazebo simulation
 *
 * @param argc number of input arguments
 * @param argv input arguments
 * @return int exit code
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "drive_param_converter");
    VESCSimulationDriver vesc_sim_driver;

    ros::spin();

    return EXIT_SUCCESS;
}