#include "laserscan_transformer.h"
/**
 * @brief Starts the laserscan transformer
 *
 * @param argc number of input arguments
 * @param argv input arguments
 * @return int exit code
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserscan_transformer");
    LaserscanTransformer tf_laserscan_to_pointcloud;
    ros::spin();

    return EXIT_SUCCESS;
}