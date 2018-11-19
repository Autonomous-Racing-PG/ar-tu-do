#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"


class CarControlLidar
{
	public:
	CarControlLidar();

	private:
 	ros::NodeHandle nh_;

	ros::Subscriber in_lidar;	
 	void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& lidar);

	ros::Publisher out_speed;
	void adjustSpeed(double speed);
};

CarControlLidar::CarControlLidar()
{
	in_lidar = nh_.subscribe<sensor_msgs::LaserScan>("/racer/laser/scan", 1, &CarControlLidar::lidar_callback, this);
 	out_speed = nh_.advertise< geometry_msgs::Twist >("/cmd_vel", 1);
}

void CarControlLidar::lidar_callback(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
	//sensor_msgs::LaserScan lidarData = lidar;
    	//std::cout << "angle_min: " << lidar->angle_min << std::endl;
	//std::cout << "range: " << lidar->ranges.size() << std::endl;

	float min = 3000;	//inf
	for(int i = 319; i < 400; i++)
	{
		if(min > lidar->ranges[i])
		min = lidar->ranges[i];
	} 

	if (min <= 0.5)
	{
		std::cout << "range: " << min << std::endl;
		geometry_msgs::Twist estop;
		estop.linear.x = 0.0;
		estop.linear.y = 0.0;
		estop.linear.z = 0.0;
		out_speed.publish(estop);
	}
}

int main(int argc, char** argv) 
{
    	ros::init(argc, argv, "car_control_lidar");
    	CarControlLidar      car_control_lidar;
	ros::spin();	
	return 0;	

}
