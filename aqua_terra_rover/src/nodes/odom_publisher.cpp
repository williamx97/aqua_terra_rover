#include "ros/ros.h"
#include <ros/package.h>
#include "nav_msgs/Odometry.h"
#include <cmath>

nav_msgs::Odometry odom;

void odomT265Callback(const nav_msgs::Odometry& msg)
{
	odom = msg;
	

}




int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "odom_publisher");
	ros::NodeHandle nodeHandle("~");

	ros::Subscriber odomT265 = nodeHandle.subscribe("/T265/odom/sample", 1, odomT265Callback);

	ros::Publisher odomPub = nodeHandle.advertise<nav_msgs::Odometry>("/odom", 1, false);
	
    ros::Rate loop_rate(1);

	while (ros::ok())
	{

        nav_msgs::Odometry msg;
        msg = odom;
        odomPub.publish(msg);

        ros::spinOnce();
		loop_rate.sleep();		
	}


	return 0;
}