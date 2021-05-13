/*
	This ROS Node sends velocity commands to make rover cover a grid of a particular size.
*/
#include "ros/ros.h"
#include <ros/package.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

float grid_size;

void gridSizeCallback(const std_msgs::Float32 & msg)
{
	grid_size = msg.data;
	//ROS_INFO_STREAM("[grid_move] Grid_Size = " << grid_size);
}


int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "grid_move");
	ros::NodeHandle nodeHandle("~");
	ros::Subscriber grid_sub = nodeHandle.subscribe("/Grid_Size", 1, gridSizeCallback);
	ros::Publisher velPub = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1, false);
    ros::Rate loop_rate(1);
	
	float output_v;
	float output_w;
	float last_gridsize=0;
	float step = 0;
	//Amount of time a velocity of 1m/s needs to be travelled to travel to end of grid
	float move_forward_time = grid_size/1.0;


	while (ros::ok())
	{
		move_forward_time = grid_size/1.0;
		ROS_INFO_STREAM("[grid_move] move_forward_time = " << move_forward_time);
		if(step<move_forward_time)
		{
			output_v = 1.0;
		}else
		{
			output_v = 0;
		}
		output_w = 0;

        //Send the velocity command to motor controllers
        geometry_msgs::Twist cmd_out;
        cmd_out.linear.x = output_v;
        cmd_out.angular.z = output_w;
        velPub.publish(cmd_out);

        ros::spinOnce();
		loop_rate.sleep();

		//Reset Steps if new grid size is read
		if(last_gridsize == grid_size)
		{
			step = step + 1.0;
		}
		else
		{
			step = 0;
		}

		last_gridsize = grid_size;
			
	}


	return 0;
}