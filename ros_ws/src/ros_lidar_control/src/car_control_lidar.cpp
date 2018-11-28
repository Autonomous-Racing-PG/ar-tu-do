#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <math.h>


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
	//std::cout << "inf: " << (typeid(lidar->ranges[0]) == typeid(float)) << std::endl;
	/*for(int i = 319; i < 400; i++)
	{
		if(min > lidar->ranges[i])
		min = lidar->ranges[i];
	} */

	/*if (min <= 0.5)
	{
		std::cout << "range: " << min << std::endl;
		geometry_msgs::Twist estop;
		estop.linear.x = 0.0;
		estop.linear.y = 0.0;
		estop.linear.z = 0.0;
		out_speed.publish(estop);
	}*/

	float left_ranges = 0;
	float right_ranges = 0;

	for(int i = 0; i < 359; i++)
	{
		if(std::isinf(lidar->ranges[i]))
		{
			right_ranges += 30;
		}
		else
		{
			right_ranges += lidar->ranges[i];
		}

	}
	for(int i = 360; i < 719; i++)
	{
		if(std::isinf(lidar->ranges[i]))
		{
			left_ranges += 30;
		}
		else
		{
			left_ranges += lidar->ranges[i];
		}

	}
	
	std::cout << "left_ranges: " << left_ranges << std::endl 
	<< "right_ranges: " << right_ranges << std::endl;

	float kp = 14;
	float kd = 0.09;

}

int main(int argc, char** argv) 
{
    	ros::init(argc, argv, "car_control_lidar");
    	CarControlLidar      car_control_lidar;
	ros::spin();	
	return 0;	

}
