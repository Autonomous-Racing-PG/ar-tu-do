#pragma once

#include "ai_config.h"

#include "fann_cpp.h"
#include "floatfann.h"

#include "ai_vector_math.h"

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

constexpr const int GENERATION_MULTIPLIER = 3;
constexpr const int GENERATION_BEST = 1;
constexpr const int GENERATION_SIZE = GENERATION_BEST + GENERATION_BEST * GENERATION_MULTIPLIER;

constexpr const float LEARNING_RATE = 0.2f; // 0 < LEARNING_RATE < 1
constexpr const float MAX_TIME = 30.0f;     // seconds

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

    std::string m_config_folder;

    // generation variables and functions
    int m_gen;
    FANN::neural_net* m_nets[GENERATION_SIZE];
    double m_scores[GENERATION_SIZE];

    FANN::neural_net* m_best_nets[GENERATION_BEST];

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
