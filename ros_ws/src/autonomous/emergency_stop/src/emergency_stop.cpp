#include "emergency_stop.h"
#include <std_msgs/Time.h>

EmergencyStop::EmergencyStop():m_emergency_status(EmergencyStatus::UNUSED), m_debug_geometry(this->m_node_handle, TOPIC_VISUALIZATION, LIDAR_FRAME)
{
    this->m_lidar_subscriber =
        this->m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &EmergencyStop::lidarCallback, this);
    this->m_emergency_stop_publisher = this->m_node_handle.advertise<std_msgs::Time>(TOPIC_EMERGENCY_STOP, 1);
    this->configureParameters();
}

void EmergencyStop::configureParameters(){
    ros::NodeHandle private_node_handle("~"); 
    
    private_node_handle.getParam(RANGE_THRESHOLD, this->m_range_threshold);
    if((this->m_range_threshold > 2.f)||(this->m_range_threshold < 0.3f))
    {
        ROS_WARN_STREAM("m_range_threshold is set to " << this->m_range_threshold 
                        << ". This value is currently limited to (0.3, 2). Using default: " 
                        << RANGE_THRESHOLD_DEFAULT << ".");
        this->m_range_threshold = RANGE_THRESHOLD_DEFAULT;
    }

    private_node_handle.getParam(MAX_RANGE, this->m_max_range);
    if((this->m_max_range > 30.f) || (this->m_max_range <= 2.f))
    {
        ROS_WARN_STREAM("m_max_range is set to " << this->m_max_range 
                        << ". This value is currently limited to [2, 30). Using default: " 
                        << MAX_RANGE_DEFAULT << ".");
        this->m_max_range = MAX_RANGE_DEFAULT;
    }
}

bool EmergencyStop::emergencyStop(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    ROS_ASSERT_MSG(this->m_range_threshold <= this->m_max_range,
                   "Threshold is bigger then max range. Function will always return false.");

    const int available_samples = (lidar->angle_max - lidar->angle_min) / lidar->angle_increment;

    // calculate the sample_angle of the lidar to be used to check if there is
    // an obstacle in front of the car, with respect to the RANGE_THRESHOLD
    // We want to check every lidar sample that is located in front of the car at a distance of RANGE_THRESHOLD.
    const float sample_angle = std::atan(CAR_BUMPER_LENGTH / 2 / this->m_range_threshold);

    const int index_start = available_samples / 2 - sample_angle / lidar->angle_increment / 2;
    const int index_end = available_samples / 2 + sample_angle / lidar->angle_increment / 2;
    ROS_ASSERT_MSG(index_start >= 0 && index_end < available_samples,
                   "sample_angle is too big. Trying to access lidar samples out of bounds.");

    // determine the minimum distance between the car and a potetial obstacle
    // Instead of calculating the range average, we are more cautious because we would
    // rather have false positives instead of false negatives.
    // i.e. we would rather stop too much than crash into an obstacle.
    const auto min_range =
        std::min(this->m_max_range, *std::min_element(lidar->ranges.begin() + index_start, lidar->ranges.begin() + index_end));
    ROS_ASSERT_MSG(min_range >= 0, "The minimal distance between the car and a potential obstacle is below zero.");

    const float CAR_BUMPER_LENGTH_HALF = CAR_BUMPER_LENGTH/2;
    //Min range to obstacle
    this->m_debug_geometry.drawLine(1, createPoint(min_range, -CAR_BUMPER_LENGTH_HALF, 0),
                                    createPoint(min_range, CAR_BUMPER_LENGTH_HALF, 0),
                                    createColor(1., 1., 0, 1.), 0.02);

    //Range threshold
    if(!(min_range < this->m_range_threshold))
    {
        this->m_debug_geometry.drawLine(0, createPoint(this->m_range_threshold, -CAR_BUMPER_LENGTH_HALF, 0),
                                    createPoint(this->m_range_threshold, CAR_BUMPER_LENGTH_HALF, 0),
                                    createColor(0, 1., 0, 1.), 0.03);
    }
    else
    {
        this->m_debug_geometry.drawLine(0, createPoint(this->m_range_threshold, -CAR_BUMPER_LENGTH_HALF, 0),
                                    createPoint(this->m_range_threshold, CAR_BUMPER_LENGTH_HALF, 0),
                                    createColor(1., 0, 0, 1.), 0.03);
    }
    

    return min_range < this->m_range_threshold;
}

void EmergencyStop::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    bool emergency_stop_active = emergencyStop(lidar);
    
    this->m_emergency_status = (this->m_emergency_status == EmergencyStatus::UNUSED) ? 
                                emergency_stop_active  ? EmergencyStatus::ACTIVATED : EmergencyStatus::CLEARED : this->m_emergency_status;

    if (emergency_stop_active)
    {
        if(this->m_emergency_status == EmergencyStatus::ACTIVATED)
        {
            ROS_INFO_STREAM("Wall detected. Emergency stop is active.");
            this->m_emergency_status = EmergencyStatus::CLEARED;
        }

        std_msgs::Time message;
        message.data = ros::Time::now();

        this->m_emergency_stop_publisher.publish(message);
    }
    else
    {
        if(this->m_emergency_status == EmergencyStatus::CLEARED)
        {
            ROS_INFO_STREAM("Emergency stop is inactive.");
            this->m_emergency_status = EmergencyStatus::ACTIVATED;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "emergency_stop");
    EmergencyStop emergency_stop;
    ros::spin();
    return EXIT_SUCCESS;
}