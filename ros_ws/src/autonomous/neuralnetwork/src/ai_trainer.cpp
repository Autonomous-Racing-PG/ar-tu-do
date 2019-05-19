#include "ai_trainer.h"

AiTrainer::AiTrainer()
{
    ros::NodeHandle private_node_handle("~");
    private_node_handle.getParam(PARAMETER_CONFIG_FOLDER, m_parameter_config_folder);
    private_node_handle.getParam(PARAMETER_TRAINING_GENERATION_MULTIPLIER, m_parameter_training_generation_multiplier);
    private_node_handle.getParam(PARAMETER_TRAINING_GENERATION_BEST, m_parameter_training_generation_best);
    m_generation_size = m_parameter_training_generation_best +
        m_parameter_training_generation_best * m_parameter_training_generation_multiplier;
    private_node_handle.getParam(PARAMETER_TRAINING_LEARNING_RATE, m_parameter_config_folder);
    private_node_handle.getParam(PARAMETER_TRAINING_MAX_TIME, m_parameter_training_max_time);

    m_crash_subscriber =
        m_node_handle.subscribe<std_msgs::Empty>(TOPIC_CRASH_SUBSCRIBE, 1, &AiTrainer::crashCallback, this);
    m_drive_parameters_subscriber =
        m_node_handle.subscribe<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS_SUBSCRIBE, 1,
                                                         &AiTrainer::driveParametersCallback, this);
    m_gazebo_model_state_publisher =
        m_node_handle.advertise<gazebo_msgs::ModelState>(TOPIC_GAZEBO_MODEL_STATE_PUBLISH, 1);
    m_net_deploy_publisher = m_node_handle.advertise<neuralnetwork::net_param>(TOPIC_NET_DEPLOY_PUBLISH, 1);

    m_nets.reserve(m_generation_size);
    m_scores.reserve(m_generation_size);
    m_best_nets.reserve(m_parameter_training_generation_best);

    initfirstGeneration();
    update();
    m_lap_timer = m_node_handle.createTimer(ros::Duration(m_parameter_training_max_time), &AiTrainer::lapTimerCallback,
                                            this, true);
    m_lap_timer.stop();
}

void AiTrainer::update()
{
    // change value 0.1
    if ((ros::Time::now() - m_time_start).toSec() < 0.1)
    {
        return;
    }

    // end previous test
    if (m_running_test)
    {
        endTest();
    }

    // create new generation if needed
    if (m_net_index == m_generation_size)
    {
        // clean and prepare next generation
        createNextGeneration();
        ROS_INFO_STREAM("successfully created generation " + std::to_string(m_gen));
        m_net_index = 0;
    }

    // deploy new test
    deploy(m_nets[m_net_index]);

    // start new test
    if (m_running_test == false)
    {
        prepareTest();
    }
}

void AiTrainer::initfirstGeneration()
{
    for (int i = 0; i < m_generation_size; i++)
    {
        FANN::neural_net* net = new FANN::neural_net;
        net->create_standard_array(NUM_LAYERS, NET_ARGS);
        net->randomize_weights((fann_type)(-1.0), (fann_type)(1.0));
        m_nets[i] = net;
        m_scores[i] = 0;
    }
    m_gen = 0;
    ROS_INFO_STREAM("created first generation randomly with " + std::to_string(m_generation_size) + " entities");
}

// TODO
void AiTrainer::chooseBestFromGeneration()
{
    for (int i = 0; i < m_parameter_training_generation_best; i++)
    {
        int best_net = -1;
        for (int j = 0; j < m_generation_size; j++)
        {
            if (best_net == -1 || m_scores[j] > m_scores[best_net])
            {
                best_net = j;
            }
        }
        m_best_nets[i] = m_nets[best_net];
        m_scores[best_net] = -1;
    }
    // save best
    m_best_nets[0]->save(m_parameter_config_folder + "/best_of/champion_" + std::to_string(m_gen) + ".config");
}

void AiTrainer::createNextGeneration()
{
    chooseBestFromGeneration();
    for (int i = 0, big_i = 0; i < m_parameter_training_generation_best; i++)
    {
        // parent
        FANN::neural_net* parent = m_best_nets[i];

        // copy parent
        m_nets[big_i] = parent;
        big_i++;

        // create mutations
        for (int i = 0; i < m_parameter_training_generation_multiplier; i++)
        {
            FANN::neural_net* child = new FANN::neural_net();
            child->create_standard_array(NUM_LAYERS, NET_ARGS);
            cloneNet(child, parent);
            mutate(child, m_parameter_training_learning_rate);
            m_nets[big_i] = child;
            big_i++;
        }
    }

    // reset scores
    for (int i = 0; i < m_generation_size; i++)
    {
        m_scores[i] = 0;
    }
    m_gen++;
}

void AiTrainer::cloneNet(FANN::neural_net* to, FANN::neural_net* from)
{
    long size = to->get_total_connections();

    FANN::connection c_from[size];
    from->get_connection_array(c_from);

    to->set_weight_array(c_from, size);
}

void AiTrainer::mutate(FANN::neural_net* net, fann_type rate)
{
    long size = net->get_total_connections();
    FANN::connection connections[size];
    net->get_connection_array(connections);

    // creating random vector
    // ============================= mutation workspace ======
    //std::vector<fann_type> mult_vec = 

    // =======================================================
    // applying vector
    //for (int i = 0; i < size; i++)
    //{
    //    connections[i].weight = connections[i].weight * mutation_delta[i];
    //}
}

void AiTrainer::lapTimerCallback(const ros::TimerEvent&)
{
    update();
}

void AiTrainer::deploy(FANN::neural_net* net)
{
    net->randomize_weights(-1, 1);

    neuralnetwork::net_param message;
    // size
    long size = net->get_total_connections();
    message.size = size;
    // connections
    FANN::connection connections[size];
    net->get_connection_array(connections);
    std::vector<unsigned int> from_neurons;
    std::vector<unsigned int> to_neurons;
    std::vector<float> weights;
    for (int i = 0; i < size; i++)
    {
        weights.push_back(connections[i].weight);
    }
    message.weights = weights;
    m_net_deploy_publisher.publish(message);
}

void AiTrainer::prepareTest()
{
    // reset car model
    gazebo_msgs::ModelState state_message;
    state_message.model_name = "racer";
    state_message.pose.position.x = 0;
    state_message.pose.position.y = 0;
    state_message.pose.position.z = 0;

    state_message.pose.orientation.x = 0;
    state_message.pose.orientation.z = 0;
    state_message.pose.orientation.w = 0;
    state_message.pose.orientation.y = 0;
    m_gazebo_model_state_publisher.publish(state_message);

    // start lap timer
    m_lap_timer.start();

    // start time counting
    m_time_start = ros::Time::now();
    m_running_test = true;

    m_speed_value = 0;
}

void AiTrainer::endTest()
{
    // stop timer
    m_lap_timer.stop();

    ros::Duration d = ros::Time::now() - m_time_start;
    ros::Time t = ros::Time(0) + d;
    m_scores[m_net_index] = m_speed_value / t.toSec();
    m_running_test = false;
    ROS_INFO_STREAM("generation: " + std::to_string(m_gen) + " | entity: " + std::to_string(m_net_index) +
                    " | score: " + std::to_string(m_scores[m_net_index]));
    m_net_index++;
}

void AiTrainer::crashCallback(const std_msgs::Empty::ConstPtr&)
{
    update();
}

void AiTrainer::driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    if (parameters->velocity < 0)
    {
        m_speed_value = 0;
        update();
    }
    m_speed_value += (double)parameters->velocity;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ai_trainer");
    AiTrainer ai_trainer;
    ros::spin();
    return EXIT_SUCCESS;
}