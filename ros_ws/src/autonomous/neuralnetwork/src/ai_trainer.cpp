#include "ai_trainer.h"

#include "ai_enum.h"
#include "ai_math.h"
#include "ai_trainer_workspace.h"
#include "ai_util.h"

using namespace ai_trainer;

AiTrainer::AiTrainer()
{
    ros::NodeHandle private_node_handle("~");
    bool load_init;
    private_node_handle.getParam(PARAMETER_LOAD_INIT, load_init);

    private_node_handle.getParam(PARAMETER_GENERATION_SIZE, m_generation_size);
    private_node_handle.getParam(PARAMETER_GENERATION_BEST, m_generation_best);
    private_node_handle.getParam(PARAMETER_LEARNING_RATE, m_learning_rate);
    private_node_handle.getParam(PARAMETER_MAX_TIME, m_max_time);

    private_node_handle.getParam(PARAMETER_CONFIG_FOLDER, m_config_folder);
    private_node_handle.getParam(PARAMETER_SAVE_LATEST_TO_INI, m_save_lastest_to_init);
    private_node_handle.getParam(PARAMETER_SAVE_GENERATION_INTERVAL, m_save_generation_interval);

    m_crash_subscriber =
        m_node_handle.subscribe<std_msgs::Empty>(TOPIC_CRASH_SUBSCRIBE, 1, &AiTrainer::crashCallback, this);
    m_drive_parameters_subscriber =
        m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS_SUBSCRIBE, 1,
                                                         &AiTrainer::driveParametersCallback, this);
    m_laptimer_subscriber =
        m_node_handle.subscribe<std_msgs::Duration>(TOPIC_LAP_TIMER_SUBSCRIBE, 1, &AiTrainer::lapTimerCallback, this);
    m_gazebo_model_state_publisher =
        m_node_handle.advertise<gazebo_msgs::ModelState>(TOPIC_GAZEBO_MODEL_STATE_PUBLISH, 1);
    m_net_deploy_publisher = m_node_handle.advertise<neuralnetwork::net_param>(TOPIC_NET_DEPLOY_PUBLISH, 1);

    if (load_init)
    {
        initLoad();
        ROS_INFO_STREAM("created first generation from init folder with " + std::to_string(m_nets.size()) + " entities");
    }
    else
    {
        init();
        ROS_INFO_STREAM("created first generation randomly with " + std::to_string(m_nets.size()) + " entities");
    }
    update();
}

void AiTrainer::update()
{
    // end previous test
    if (m_running_test)
    {
        meta* m = m_meta[m_index]; // read only
        endTest();
        std::string output = ai_workspace::get_test_output(m, m_gen, m_index, m_nets.size());
        ROS_INFO_STREAM(output);
        m_index++;
    }

    // create new generation if needed
    if (m_index == m_nets.size())
    {
        chooseBest();
        createNextGeneration();
        ROS_INFO_STREAM("==========================================================");
        // if(m_gen > 3)
        //{
        //    m_learning_rate = 1.0 / m_gen * 3;
        //}
        ROS_INFO_STREAM("generation: " + std::to_string(m_gen) + " | learning_rate: " +
                        std::to_string(m_learning_rate));
        m_index = 0;
    }
    // start new test
    if (m_running_test == false)
    {
        deploy(m_nets[m_index]);
        prepareTest();
    }
}

bool AiTrainer::init()
{
    m_nets.clear();
    m_meta.clear();
    for (int i = 0; i < m_generation_size; i++)
    {
        FANN::neural_net* net = new FANN::neural_net();
        net->create_standard_array(ai_workspace::DEFAULT_NUM_LAYERS, ai_workspace::DEFAULT_LAYER_ARRAY);
        m_nets.push_back(net);
        m_meta.push_back(new meta());
    }
    m_gen = 0;
    m_index = 0;
    return true;
}

bool AiTrainer::initLoad()
{
    m_nets.clear();
    m_meta.clear();

    std::string init_folder = m_config_folder + "/" + PATH_INIT_FOLDER;
    std::vector<std::string> paths = ai_util::get_files_in_folder(init_folder);
    if (paths.size() == 0)
    {
        ROS_WARN_STREAM("could not load from \"" + init_folder + "\"");
        return false;
    }

    // adding networks
    for (uint i = 0; i < paths.size(); i++)
    {
        std::string p = paths[i];
        FANN::neural_net* net = new FANN::neural_net();
        bool b = net->create_from_file(p);
        if (b)
        {
            m_nets.push_back(net);
            m_meta.push_back(new meta());
        }
        else
        {
            ROS_WARN_STREAM("could not load " + p);
        }
    }
    m_gen = 0;
    m_index = 0;
    return true;
}

// TODO
void AiTrainer::chooseBest()
{
    m_best_nets.clear();

    // clear choosen attribute
    for (uint i = 0; i < m_meta.size(); i++)
    {
        m_meta[i]->choosen = false;
    }

    bool b = true;
    while (b && m_best_nets.size() < (uint)m_generation_best)
    {
        // find best index
        int best_i = -1;
        for (uint i = 0; i < m_nets.size(); i++)
        {
            meta* m = m_meta[i];
            if (m->choosen == false)
            {
                if (best_i == -1 || m->score > m_meta[best_i]->score)
                {
                    best_i = i;
                }
            }
        }
        if (best_i != -1)
        {
            FANN::neural_net* best = m_nets[best_i];
            m_best_nets.push_back(best);
            m_meta[best_i]->choosen = true;

            // reset score
            m_meta[best_i]->score = 0;
        }
        else
        {
            // no more left to choose
            b = false;
        }
    }

    // best choosen

    if (m_save_generation_interval != 0 && m_gen != 0 && m_gen % m_save_generation_interval == 0)
    {
        // save to best_of
        for (uint i = 0; i < m_best_nets.size(); i++)
        {
            std::string path =
                m_config_folder + "/best_of/s" + std::to_string(m_gen) + "e" + std::to_string(i) + ".conf";
            bool b = m_best_nets[i]->save(path);
            if (!b)
            {
                ROS_WARN_STREAM("could not save to: " + path);
            }
        }
    }

    {
        // save lastest to config
        std::string path = m_config_folder + "/latest.conf";
        bool b = m_best_nets[0]->save(path);
        if (!b)
        {
            ROS_WARN_STREAM("could not save to: " + path);
        }
    }
    if (m_save_lastest_to_init)
    {
        // save lastest to config/init
        std::string path = m_config_folder + "/init/latest.conf";
        bool b = m_best_nets[0]->save(path);
        if (!b)
        {
            ROS_WARN_STREAM("could not save to: " + path);
        }
    }
}

void AiTrainer::createNextGeneration()
{
    m_nets.clear();
    m_meta.clear();

    // copy parents
    for (uint i = 0; i < m_best_nets.size(); i++)
    {
        m_nets.push_back(m_best_nets[i]);
        m_meta.push_back(new meta());
    }

    // create mutations
    int index = 0;
    while (m_nets.size() < (size_t)m_generation_size)
    {
        using namespace ai_math;

        FANN::neural_net* parent = m_best_nets[index];

        NetVector parent_vec = net_to_vector(parent);
        NetVector m_vec = ai_workspace::mutate(parent_vec, m_learning_rate);
        uint layers = parent->get_num_layers();
        uint layer_array[parent->get_num_layers()];
        parent->get_layer_array(layer_array);
        FANN::neural_net* m = vector_to_net(m_vec, layers, layer_array);

        m_nets.push_back(m);
        m_meta.push_back(new meta());
        index = (index + 1) % m_best_nets.size();
    }
    m_gen++;
}

void AiTrainer::deploy(FANN::neural_net* net)
{
    uint layers = net->get_num_layers();
    std::vector<uint> layer_array(layers);
    net->get_layer_array(&layer_array[0]);

    uint weight_array_size = net->get_total_connections();
    FANN::connection from_weight_array[weight_array_size];
    net->get_connection_array(from_weight_array);
    std::vector<float> weight_array;
    for (uint i = 0; i < weight_array_size; i++)
    {
        weight_array.push_back(from_weight_array[i].weight);
    }

    neuralnetwork::net_param message;
    message.layers = layers;
    message.layer_array = layer_array;
    message.weight_array_size = weight_array_size;
    message.weight_array = weight_array;
    m_net_deploy_publisher.publish(message);
}

void AiTrainer::prepareTest()
{
    std::srand(std::time(nullptr)); // use current time as seed for random generator

    // float pos_x[] = {-0.4, 4, 12, 16};
    // -0.4 4 12 16

    // reset car model
    gazebo_msgs::ModelState state_message;
    state_message.model_name = "racer";
    state_message.pose.position.x = 0;
    state_message.pose.position.y = -0.4; // pos_x[std::rand() % 4];
    state_message.pose.position.z = 0;

    state_message.pose.orientation.x = 0;
    state_message.pose.orientation.z = std::rand() % 2;
    state_message.pose.orientation.w = 0;
    state_message.pose.orientation.y = 0;
    m_gazebo_model_state_publisher.publish(state_message);

    // start lap timer
    m_timer = m_node_handle.createTimer(ros::Duration(m_max_time), &AiTrainer::timerCallback, this, true);

    // start time counting
    m_meta[m_index]->time_start = ros::Time::now();

    m_running_test = true;
}

void AiTrainer::endTest()
{
    meta* m = m_meta[m_index];
    m->run_time = (ros::Time::now() - m->time_start).toSec();
    m->avg_velocity = m->added_velocity / (double)m->count;
    m->score = ai_workspace::fitness(m);

    m_running_test = false;
}

void AiTrainer::timerCallback(const ros::TimerEvent&)
{
    meta* m = m_meta[m_index];
    if (ai_workspace::event(m, ai_enum::AbortReason::max_run_time))
    {
        update();
    }
}

void AiTrainer::crashCallback(const std_msgs::Empty::ConstPtr&)
{
    meta* m = m_meta[m_index];

    // remove magic number 0.1
    if ((ros::Time::now() - m->time_start).toSec() < 0.1)
    {
        return;
    }

    if (ai_workspace::event(m, ai_enum::AbortReason::crash))
    {
        update();
    }
}

void AiTrainer::driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    meta* m = m_meta[m_index];

    m->c_velocity = (double)parameters->velocity; // latest velocity publish
    m->c_angle = (double)parameters->angle;       // latest angle publish

    m->added_velocity += (double)parameters->velocity;
    m->added_angle += std::abs((double)parameters->angle);
    m->count++;

    if (ai_workspace::event(m, ai_enum::AbortReason::output))
    {
        update();
    }
}

void AiTrainer::lapTimerCallback(const std_msgs::Duration::ConstPtr& time_message)
{
    meta* m = m_meta[m_index];
    m->lap_time = time_message->data.toSec();

    if (ai_workspace::event(m, ai_enum::AbortReason::lap_finished))
    {
        update();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ai_trainer");
    AiTrainer ai_trainer;
    ros::spin();
    return EXIT_SUCCESS;
}
