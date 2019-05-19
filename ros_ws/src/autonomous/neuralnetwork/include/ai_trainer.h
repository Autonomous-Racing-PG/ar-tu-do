#pragma once

#include "ai_config.h"

// http://leenissen.dk/fann/html/files/fann_cpp-h.html
// clang-format off
#include "floatfann.h"
#include "fann_cpp.h"
// clang-format on

#include <dirent.h>
#include <map>
#include <sys/stat.h>
#include <vector>

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <drive_msgs/drive_param.h>
#include <gazebo_msgs/ModelState.h>
#include <neuralnetwork/net_param.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

constexpr const char* TOPIC_CRASH_SUBSCRIBE = "/crash";
constexpr const char* TOPIC_DRIVE_PARAMETERS_SUBSCRIBE = "/commands/drive_param";

constexpr const char* TOPIC_GAZEBO_MODEL_STATE_PUBLISH = "/gazebo/set_model_state";
constexpr const char* TOPIC_NET_DEPLOY_PUBLISH = "/nn/net/deploy";

constexpr const char* PARAMETER_CONFIG_FOLDER = "nn_config_folder";

constexpr const char* PARAMETER_TRAINING_GENERATION_MULTIPLIER = "nn_training_generation_multiplier";
constexpr const char* PARAMETER_TRAINING_GENERATION_BEST = "nn_training_generation_best";
constexpr const char* PARAMETER_TRAINING_LEARNING_RATE = "nn_training_learning_rate";
constexpr const char* PARAMETER_TRAINING_MAX_TIME = "nn_training_max_time";

class AiTrainer
{

    public:
    AiTrainer();

    private:
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_crash_subscriber;
    ros::Subscriber m_drive_parameters_subscriber;
    ros::Publisher m_gazebo_model_state_publisher;
    ros::Publisher m_net_deploy_publisher;

    std::string m_parameter_config_folder;
    int m_parameter_training_generation_multiplier;
    int m_parameter_training_generation_best;
    int m_generation_size;
    fann_type m_parameter_training_learning_rate;
    float m_parameter_training_max_time;

    // generation variables and functions
    int m_gen;
    std::vector<FANN::neural_net*> m_nets;
    std::vector<double> m_scores;

    std::vector<FANN::neural_net*> m_best_nets;

    void update();

    // generations functions
    void initfirstGeneration();
    void chooseBestFromGeneration();
    void createNextGeneration();
    void cloneNet(FANN::neural_net* to, FANN::neural_net* from);

    void mutate(FANN::neural_net* net, fann_type rate);

    // net variables and functions
    int m_net_index = 0;

    void prepareTest();
    void endTest();

    // current test
    bool m_running_test = false;
    ros::Time m_time_start;
    double m_speed_value;
    ros::Timer m_lap_timer;
    void lapTimerCallback(const ros::TimerEvent&);

    void deploy(FANN::neural_net* net);

    // callbacks
    void crashCallback(const std_msgs::Empty::ConstPtr&);
    void driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);
};
