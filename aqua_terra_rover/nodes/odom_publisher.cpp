#include "ros/ros.h"
#include <ros/package.h>
#include "nav_msgs/Odometry"
#include <cmath>

nav_msgs::Odometry odom;

void odomT265Callback(const nav_msgs::Odometry& msg)
{
	odom = msg;
	

}

}


int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "inner_controller");
	ros::NodeHandle nodeHandle("~");

	ros::Subscriber sub_theta_d_r = nodeHandle.subscribe("/groupA/r_wheel_encoder/theta_d_r", 1, theta_d_rCallback);
	ros::Subscriber sub_theta_d_l = nodeHandle.subscribe("/groupA/l_wheel_encoder/theta_d_l", 1, theta_d_lCallback);
	
	ros::Subscriber odomT265 = nodeHandle.subscribe("/groupA/trajectory_tracking/cmd_vel", 1, velocityCallback);

	ros::Publisher odomPub = nodeHandle.advertise<nav_msgs::Odometry>("/odom", 1, false);
	
    ros::Rate loop_rate(1);

	while (ros::ok())
	{


		deb1.data = errorThetaRdot;
		deb2.data = left_PWM;
		pub_deb_1.publish(deb1);
		pub_deb_2.publish(deb2);
		
        ros::spinOnce();
		loop_rate.sleep();		
	}


	return 0;
}