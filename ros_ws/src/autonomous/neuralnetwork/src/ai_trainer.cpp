#include "ai_trainer.h"

AiTrainer::AiTrainer()
{
    ros::NodeHandle private_node_handle("~");
    private_node_handle.getParam(PARAMETER_CONFIG_FOLDER, m_config_folder);
    m_crash_subscriber =
        m_node_handle.subscribe<std_msgs::Empty>(TOPIC_CRASH_SUBSCRIBE, 1, &AiTrainer::crashCallback, this);
    m_gazebo_model_state_publisher =
        m_node_handle.advertise<gazebo_msgs::ModelState>(TOPIC_GAZEBO_MODEL_STATE_PUBLISH, 1);
    m_net_deploy_publisher = m_node_handle.advertise<neuralnetwork::net_param>(TOPIC_NET_DEPLOY_PUBLISH, 1);

    initfirstGeneration();
    update();
}

void AiTrainer::update()
{
    // end previous test
    if (m_running_test)
    {
        endTest();
    }

    // create new generation if needed
    if (m_net_index == GENERATION_SIZE)
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
    for (int i = 0; i < GENERATION_SIZE; i++)
    {
        FANN::neural_net* net = new FANN::neural_net;
        net->create_standard_array(NUM_LAYERS, NET_ARGS);
        net->randomize_weights((fann_type)(-1.0), (fann_type)(1.0));
        m_nets[i] = net;
        m_scores[i] = 0;
    }
    m_gen = 0;
    ROS_INFO_STREAM("created first generation randomly with " + std::to_string(GENERATION_SIZE) + " entities");
}

// TODO
void AiTrainer::chooseBestFromGeneration()
{
    for (int i = 0; i < GENERATION_BEST; i++)
    {
        int best_net = -1;
        for (int j = 0; j < GENERATION_SIZE; j++)
        {
            if (best_net == -1 || m_scores[j] > m_scores[best_net])
            {
                best_net = j;
            }
        }
        m_best_nets[i] = m_nets[best_net];
        m_scores[best_net] = -1;
    }
}

void AiTrainer::createNextGeneration()
{
    chooseBestFromGeneration();
    for (int i = 0, big_i = 0; i < GENERATION_BEST; i++)
    {
        // parent
        FANN::neural_net* parent = m_best_nets[i];

        // copy parent
        m_nets[big_i] = parent;
        big_i++;

        // create parent mutations
        for (int i = 0; i < GENERATION_MULTIPLIER; i++)
        {
            FANN::neural_net* child = new FANN::neural_net();
            child->create_standard_array(NUM_LAYERS, NET_ARGS);
            cloneNet(child, parent);
            mutate(child, 1);
            m_nets[big_i] = child;
            big_i++;
        }
    }

    // reset scores
    for (int i = 0; i < GENERATION_SIZE; i++)
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
    std::vector<fann_type> mutation_delta = generateRandomVector(size, rate);

    // applying vector
    for (int i = 0; i < size; i++)
    {
        connections[i].weight = connections[i].weight * mutation_delta[i];
    }
}

std::vector<fann_type> AiTrainer::generateRandomVector(int size, fann_type rate)
{
    std::vector<fann_type> vec;
    int i = 0;
    fann_type randValue = 0;
    srand(time(NULL));
    while (i < size)
    {
        randValue = (fann_type)0.5 + (rand() * rate);
        vec.push_back(randValue);
        i++;
    }
    return vec;
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

    // start timer
    m_time_start = ros::Time::now();
    m_running_test = true;
}

void AiTrainer::endTest()
{
    ros::Duration d = ros::Time::now() - m_time_start;
    ros::Time t = ros::Time(0) + d;
    m_scores[m_net_index] = t.toSec();
    m_running_test = false;
    ROS_INFO_STREAM("test ended | generation: " + std::to_string(m_gen) + " | entity: " + std::to_string(m_net_index) +
                    " | score: " + std::to_string(m_scores[m_net_index]));
    m_net_index++;
}

void AiTrainer::crashCallback(const std_msgs::Empty::ConstPtr&)
{
    update();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ai_trainer");
    AiTrainer ai_trainer;
    ros::spin();
    return EXIT_SUCCESS;
}