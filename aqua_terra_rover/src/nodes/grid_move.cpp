/*
	This ROS Node sends pose commands to make rover cover a grid of a particular size.
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
#include "std_msgs/Bool.h"
#include <cmath>

/*GLOBAL VARIABLES START**************************************************************************************************/
int step = 0;
bool is_robot_at_goal_pose = false;
/*GLOBAL VARIABLES STOP**************************************************************************************************/

void goalCallBack(const std_msgs::Bool & msg)
{
	is_robot_at_goal_pose = msg.data;
}

int main(int argc, char* argv[])
{

	/*INITALIZATION START**************************************************************************************************/
	ros::init(argc, argv, "grid_move");
	ros::NodeHandle nodeHandle("~");
	ros::Subscriber goal_sub = nodeHandle.subscribe("/point_move/is_robot_at_goal_pose", 1, goalCallBack);
	ros::Publisher posePub = nodeHandle.advertise<geometry_msgs::Pose2D>("/goalPose", 1, false);
    ros::Rate loop_rate(40);
	/*INITALIZATION STOP**************************************************************************************************/

	while (ros::ok())
	{	
		//Tick 
		if(step > 3)
		{
			step = 0;
		}
		geometry_msgs::Pose2D output_goal;

		if(is_robot_at_goal_pose && step == 0)
		{
			output_goal.x = 0;
			output_goal.y = 0;
			output_goal.theta = 0;
			ROS_INFO_STREAM("[GRID MOVE] Moving to 0,0 ");
			step = step + 1;
		}
		else if (is_robot_at_goal_pose && step == 1)
		{
			output_goal.x = 1;
			output_goal.y = 0;
			output_goal.theta = -1.57;	
			ROS_INFO_STREAM("[GRID MOVE] Moving to 1,0 ");	
			step = step + 1;
		}
		else if(is_robot_at_goal_pose && step == 2)
		{
			output_goal.x = 1;
			output_goal.y = 1;
			output_goal.theta = -3.14;
			ROS_INFO_STREAM("[GRID MOVE] Moving to 1,1 ");
			step = step + 1;
		}
		else if(is_robot_at_goal_pose && step == 3)
		{
			output_goal.x = 0;
			output_goal.y = 1;
			output_goal.theta = 1.57;
			ROS_INFO_STREAM("[GRID MOVE] Moving to 0,1 ");
			step = step + 1;
		}
		
		posePub.publish(output_goal);
        ros::spinOnce();
		loop_rate.sleep();
		
	}


	return 0;
}