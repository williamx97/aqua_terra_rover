/*
	This ROS Node sends velocity commands to make rover cover a grid of a particular size.
*/
#include "ros/ros.h"
#include "ros/package.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "tf/transform_datatypes.h"
#include <cmath>

/*GLOBAL VARIABLES START**************************************************************************************************/
nav_msgs::Odometry odometry;
geometry_msgs::Pose2D goal_pose;
float error_distance;
float error_heading;
float goal_xy_heading;
float error_xy_heading;
float gain_heading = 0.5;
float gain_distance = 0.1;
float gain_xy_heading = 0.5;
float output_v = 0;
float output_w = 0;
int alg_case;
/*GLOBAL VARIABLES STOP**************************************************************************************************/

void odomCallback(const nav_msgs::Odometry & msg)
{
	odometry = msg;
}

void goalPoseCallback(const geometry_msgs::Pose2D & msg)
{
	goal_pose = msg;
}


int main(int argc, char* argv[])
{

	/*INITALIZATION START**************************************************************************************************/
	ros::init(argc, argv, "point_move");
	ros::NodeHandle nodeHandle("~");
	ros::Subscriber odom_sub = nodeHandle.subscribe("/odom", 1, odomCallback);
	ros::Subscriber goal_pose_sub = nodeHandle.subscribe("/goalPose", 1, goalPoseCallback);
	ros::Publisher velPub = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1, false);
	//Loop rate of 1000Hz
    ros::Rate loop_rate(40);
	/*INITALIZATION STOP**************************************************************************************************/

	while (ros::ok())
	{	;;
		float odom_x = odometry.pose.pose.position.x;
		float odom_y = odometry.pose.pose.position.y;
		float goal_x = goal_pose.x;
		float goal_y = goal_pose.y;
		float goal_heading = goal_pose.theta;

		/*Need To convert from quaternions to roll,pitch,yaw(yaw is heading)*/
		tf::Quaternion q(
            odometry.pose.pose.orientation.x, 
            odometry.pose.pose.orientation.y, 
            odometry.pose.pose.orientation.z, 
            odometry.pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, odom_heading;
		m.getRPY(roll, pitch, odom_heading);
		
        // ROS_INFO_STREAM("[POINT MOVE] x direction is " << odom_x);
        // ROS_INFO_STREAM("[POINT MOVE] y direction is " << odom_y);
        // ROS_INFO_STREAM("[POINT MOVE] theta direction is " << odom_heading);

        /*ERROR CALCULATIONS START********************************************************************************************/
        //Calculate how far away the current [x,y] is from the goal [x,y] (Distance Between 2 points)
        error_distance = sqrt( pow(goal_x-odom_x,2) + pow(goal_y-odom_y,2) );
        //Calculate the error in the current pose and goal pose in theta.
        error_heading = goal_pose.theta - odom_heading;
        //Caclulate where the [x,y] heading is
        goal_xy_heading = (float)atan2((double)goal_y-odom_y,(double)goal_x-odom_x);
        //Calculate the error in [x,y] heading and current heading
        error_xy_heading = goal_xy_heading - odom_heading;

        //Accounting for non-linearity in -pi and pi
        if(error_heading > 3.14159265358979323846)
        {
            error_heading = error_heading - 2*3.14159265358979323846;
        }
        if(error_distance <-3.14159265358979323846)
        {
            error_heading = error_heading + 2*3.14159265358979323846;
        }

        if(error_xy_heading > 3.14159265358979323846)
        {
            error_xy_heading = error_xy_heading - 2*3.14159265358979323846;
        }
        if(error_xy_heading <-3.14159265358979323846)
        {
            error_xy_heading = error_xy_heading + 2*3.14159265358979323846;
        }
		/*ERROR CALCULATION STOP**********************************************************************************************/
		
		
		//Algorithm Overview
        //1)If not at [x,y] and not facing [x,y]          =publish [w] only 
        //2)If not at [x,y] and facing [x,y]              =publish [v] only 
        //3)If at [x,y] and not facing goal [theta]       =publish [w] only 
        //4)If at [x,y] and facing goal [theta]           =publish [v]=[w]=0

        if(abs(error_distance) > 0.35 && abs(error_xy_heading) > 0.2)
        {
            alg_case = 1;
        }
        else if (abs(error_distance) > 0.35 && abs(error_xy_heading) < 0.2)
        {
            alg_case = 2;
        }
        else if (abs(error_distance) < 0.35 && abs(error_heading) > 0.2)
        {
            alg_case = 3;
        }
        else
        {
            alg_case = 4;
        }

        switch(alg_case) {
            case 1:
                output_w = gain_xy_heading * error_xy_heading ;
                output_v = 0;   
            break;

            case 2:
                output_v = gain_distance * error_distance;
                output_w = 0;
            break;

            case 3:
                output_w = gain_heading * error_heading;
                ROS_INFO_STREAM("[POINT MOVE] my heading error is " << error_heading);
                ROS_INFO_STREAM("[POINT MOVE] my gain is " << gain_heading);
                output_v = 0;
            break;

            default:           
                output_w = 0;
                output_v = 0;
        }

        //Set a minimum rotational Speed
        if(abs(output_w) < 0.6 && output_w != 0)
        {
            
            if(output_w<0)
            {
                output_w = -0.6;
            }else{
                output_w = 0.6;
            }
        }

        //Set a maxiumum rotational Speed
        if(abs(output_w) > 1.2 && output_w != 0)
        {
            
            if(output_w<0)
            {
                output_w = -1.2;
            }else{
                output_w = 1.2;
            }
        }

        // ROS_INFO_STREAM("[POINT MOVE] Moving with [v] " << output_v);
        // ROS_INFO_STREAM("[POINT MOVE] Moving with [w] " << output_w);
        //Send the velocity command to motor controllers
        geometry_msgs::Twist cmd_out;
        cmd_out.linear.x = output_v;
        cmd_out.angular.z = output_w;
        velPub.publish(cmd_out);

        ros::spinOnce();
		loop_rate.sleep();
		
	}


	return 0;
}