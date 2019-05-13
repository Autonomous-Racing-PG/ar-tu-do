#pragma once

#include "floatfann.h"
#include "fann_cpp.h"

#include <dirent.h>
#include <map>
#include <sys/stat.h>
#include <vector>

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <drive_msgs/drive_param.h>
#include <gazebo_msgs/ModelState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

constexpr const char* TOPIC_CRASH_SUBSCRIBE = "/crash";

constexpr const char* TOPIC_NN_LOAD_PUBLISH = "/nn/load";
constexpr const char* TOPIC_NN_BATCH_LOAD_PUBLISH = "/nn/batch/load";
constexpr const char* TOPIC_NN_BATCH_SELECT_PUBLISH = "/nn/batch/select";
constexpr const char* TOPIC_NN_STATUS_PUBLISH = "/nn/status/set";

constexpr const char* TOPIC_GAZEBO_MODEL_STATE_PUBLISH = "/gazebo/set_model_state";

constexpr const char* PARAMETER_CONFIG_PATH = "config_path";

constexpr const int GENERATION_MULTIPLIER = 10;
constexpr const int GENERATION_BEST = 10;
constexpr const int GENERATION_SIZE = GENERATION_BEST + GENERATION_BEST * GENERATION_MULTIPLIER;

class AiTrainer
{
    public:
    AiTrainer();
    void spin();

    private:
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_crash_subscriber;
    ros::Publisher m_nn_load_publisher;
    ros::Publisher m_nn_batch_load_publisher;
    ros::Publisher m_nn_batch_select_publisher;
    ros::Publisher m_nn_status_publisher;
    ros::Publisher m_gazebo_model_state_publisher;

    void update();

    // generation variables and functions
    int m_gen = 0;
    int m_scores[GENERATION_SIZE];

    int createNextGeneration(int current_gen);

    // current test variables and functions
    int m_entity = 0;
    ros::Time time_start;
    //
    std::string m_config_path;

    void prepare();
    void end();

    void mutate(FANN::neural_net& net, fann_type rate);
    std::vector<fann_type> generateRandomVector(int size, fann_type rate);

    void crashCallback(const std_msgs::Empty::ConstPtr&);

    void setStatus(bool status);
    void loadConfig(std::string name);

    void loadBatch(std::string batch);
    void selectBatch(std::string name);
};
