#include "car_config.h"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <ros/package.h>

int main(int argc, char** argv)
{
    YAML::Node node;
    node["speed_to_erpm_gain"] = car_config::SPEED_TO_ERPM;
    node["speed_to_erpm_offset"] = 0;
    node["steering_angle_to_servo_gain"] = car_config::STEERING_TO_SERVO_GAIN;
    node["steering_angle_to_servo_offset"] = car_config::STEERING_TO_SERVO_OFFSET;
    node["wheelbase"] = car_config::WHEELBASE;

    std::string package_path = ros::package::getPath("vesc_sim");
    std::ofstream fout(package_path + "/config/car_config.yaml");
    fout << node;
}
