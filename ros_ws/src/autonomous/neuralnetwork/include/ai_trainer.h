#pragma once

#include "ai_enum.h"

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
#include <std_msgs/Duration.h>

namespace ai_trainer
{
    constexpr const char* TOPIC_CRASH_SUBSCRIBE = "/crash";
    constexpr const char* TOPIC_DRIVE_PARAMETERS_SUBSCRIBE = "/commands/drive_param";
    constexpr const char* TOPIC_LAP_TIMER_SUBSCRIBE = "/lap_time";

    constexpr const char* TOPIC_GAZEBO_MODEL_STATE_PUBLISH = "/gazebo/set_model_state";
    constexpr const char* TOPIC_NET_DEPLOY_PUBLISH = "/ai/deploy";

    constexpr const char* PARAMETER_LOAD_INIT = "load_init";
    constexpr const char* PARAMETER_CONFIG_FOLDER = "config_folder";
    constexpr const char* PARAMETER_SAVE_LATEST_TO_INI = "save_lastest_to_init";
    constexpr const char* PARAMETER_SAVE_GENERATION_INTERVAL = "save_generation_interval";

    constexpr const char* PARAMETER_GENERATION_SIZE = "generation_size";
    constexpr const char* PARAMETER_GENERATION_BEST = "generation_best";
    constexpr const char* PARAMETER_LEARNING_RATE = "learning_rate";
    constexpr const char* PARAMETER_MAX_TIME = "max_time";

    constexpr const char* PATH_INIT_FOLDER = "init";

    struct TrainingContext
    {
        // variable only valid in or after the folliwong phases:

        // test start
        ros::Time time_start = ros::Time::now();

        // during test
        double cumulative_velocity = 0;
        double cumulative_angle = 0;
        uint count = 0;

        double c_velocity = 0; // latest velocity publish
        double c_angle = 0;    // latest angle publish

        // test end
        ai_enum::EventCause abort_cause = ai_enum::EventCause::NONE;
        std::string abort_cause_details = "";
        double run_time = 0;
        double avg_velocity = 0;
        double lap_time = 0;

        double score = 0;
        bool choosen = false;
    };

    class AiTrainer
    {
        public:
        AiTrainer();
        void spin();

        private:
        ros::NodeHandle m_node_handle;
        ros::Subscriber m_crash_subscriber;
        ros::Subscriber m_drive_parameters_subscriber;
        ros::Subscriber m_laptimer_subscriber;
        ros::Publisher m_gazebo_model_state_publisher;
        ros::Publisher m_net_deploy_publisher;

        bool m_save_lastest_to_init;
        int m_save_generation_interval;

        std::string m_config_folder;
        int m_generation_size;
        int m_generation_best;
        fann_type m_learning_rate;
        float m_max_time;

        // generation variables and functions
        int m_gen;
        std::vector<FANN::neural_net*> m_nets;
        std::vector<TrainingContext*> m_training_contexts;

        std::vector<FANN::neural_net*> m_best_nets;

        void update();

        // generations functions

        bool init();
        bool initLoad();
        void chooseBest();
        void createNextGeneration();

        // net variables and functions
        uint m_index = 0;

        void prepareTest();
        void endTest();

        // current test
        bool m_running_test = false;
        ros::Timer m_timer;

        void deploy(FANN::neural_net* net);

        // callbacks
        void timerCallback(const ros::TimerEvent&);
        void crashCallback(const std_msgs::Empty::ConstPtr&);
        void driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);
        void lapTimerCallback(const std_msgs::Duration::ConstPtr& time);
    };
}