#include "ai_trainer.h"

AiTrainer::AiTrainer()
{
    ros::NodeHandle private_node_handle("~");
    private_node_handle.getParam(PARAMETER_CONFIG_PATH, m_config_path);
    m_crash_subscriber =
        m_node_handle.subscribe<std_msgs::Empty>(TOPIC_CRASH_SUBSCRIBE, 1, &AiTrainer::crashCallback, this);
    m_nn_load_publisher = m_node_handle.advertise<std_msgs::String>(TOPIC_NN_LOAD_PUBLISH, 1);
    m_nn_batch_load_publisher = m_node_handle.advertise<std_msgs::String>(TOPIC_NN_BATCH_LOAD_PUBLISH, 1);
    m_nn_batch_select_publisher = m_node_handle.advertise<std_msgs::String>(TOPIC_NN_BATCH_SELECT_PUBLISH, 1);
    m_nn_status_publisher = m_node_handle.advertise<std_msgs::Bool>(TOPIC_NN_STATUS_PUBLISH, 1);
    m_gazebo_model_state_publisher =
        m_node_handle.advertise<gazebo_msgs::ModelState>(TOPIC_GAZEBO_MODEL_STATE_PUBLISH, 1);
    ROS_INFO_STREAM("ai trainer created");

    // load first generation from disk
    loadBatch(std::to_string(m_gen));
}

void AiTrainer::prepare()
{
    ROS_INFO_STREAM("launching ai");
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

    // create new ai?
    // setting start time
    time_start = ros::Time::now();
}

void AiTrainer::end()
{
    ros::Duration d = ros::Time::now() - time_start;
    ros::Time t = ros::Time(0) + d;
    ROS_INFO_STREAM("time survived: " + std::to_string(t.toSec()));
    ROS_INFO_STREAM("========================");
}

void AiTrainer::update()
{
    // clean entity
    if (m_entity == GENERATION_SIZE || m_scores[m_entity] == -1)
    {
        // clean and prepare next generation
        ROS_INFO_STREAM("creating generation " + std::to_string(m_gen + 1));
        int s = createNextGeneration(m_gen);
        m_gen++;
        ROS_INFO_STREAM("successfully created generation " + std::to_string(m_gen) + " with " + std::to_string(s) +
                        " entities");
        loadBatch(std::to_string(m_gen));
        m_entity = 0;
    }

    // prepare entity
    prepare();
    selectBatch(std::to_string(m_entity));
}

int AiTrainer::createNextGeneration(int current_gen)
{
    int next_gen = current_gen + 1;
    std::string in = m_config_path + "/" + std::to_string(current_gen) + std::to_string(current_gen);
    std::string out = m_config_path + "/" + std::to_string(next_gen);
    // mkdir(out.c_str(), 0);

    int index = 0;

    DIR* dir;
    struct dirent* ent;
    if ((dir = opendir(in.c_str())) != NULL)
    {
        while ((ent = readdir(dir)) != NULL)
        {
            std::string name = ent->d_name;
            if (name.compare(".") == 0 || name.compare("..") == 0)
            {
                continue;
            }
            std::string in_file = in + "/" + name;
            FANN::neural_net net;
            bool ret = net.create_from_file(in_file);
            if (ret == false)
            {
                ROS_WARN_STREAM("error loading config:" + in_file);
            }
            else
            {
                // copy parent
                net.save(out + "/" + std::to_string(index) + ".net");
                index++;
                // create and saving offspring
                for (int i = 0; i < GENERATION_MULTIPLIER; i++)
                {
                    mutate(net, 1);
                    net.save(out + "/" + std::to_string(index) + ".net");
                    index++;
                }
            }
        }
    }
    closedir(dir);

    // fill not used scores
    for (int i = 0; i < GENERATION_SIZE; i++)
    {
        m_scores[i] = i < index ? 0 : -1;
    }

    return index;
}

void AiTrainer::mutate(FANN::neural_net& net, fann_type rate)
{
    long size = net.get_total_connections();
    FANN::connection connections[size];
    net.get_connection_array(connections);

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

void AiTrainer::crashCallback(const std_msgs::Empty::ConstPtr&)
{
    update();
}

void AiTrainer::setStatus(bool status)
{
    std_msgs::Bool status_message;
    status_message.data = status;
    m_nn_status_publisher.publish(status_message);
}

void AiTrainer::loadConfig(std::string name)
{
    std_msgs::String load_message;
    load_message.data = name;
    m_nn_load_publisher.publish(load_message);
}

void AiTrainer::loadBatch(std::string batch)
{
    std_msgs::String batch_message;
    batch_message.data = batch;
    m_nn_batch_load_publisher.publish(batch_message);
}

void AiTrainer::selectBatch(std::string name)
{
    std_msgs::String select_message;
    select_message.data = name;
    m_nn_batch_select_publisher.publish(select_message);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ai_trainer");
    AiTrainer ai_trainer;
    ros::spin();
    return EXIT_SUCCESS;
}
