# aqua_terra_rover

This will be the ROS Package that contains all the custom code to be executed on the rover.

## Prerequisite Knowledge  
* Python Tutorial: https://www.learnpython.org/
* C++ Tutorial: https://www.learn-cpp.org/en/Welcome
* GitHub Tutorials (Do all First Day on Github Courses): https://lab.github.com/
* ROS Tutorial (Complete all beginner level tutorials): http://wiki.ros.org/ROS/Tutorials
* You must also know how to navigate files systems via a Linux Terminal

## Prerequisite Software
All below packages must be installed on the Turtle Rover prior to installing the packages in this repository
* Turtle Rover ROS Packages [Turtle Rover ROS Packages](https://github.com/TurtleRover/tr_ros)
* Intel Realsense SDK 'librealsense' https://github.com/IntelRealSense/librealsense
* Intel Realsense ROS Wrapper packages [Intel Realsense ROS Wrapper](https://github.com/IntelRealSense/realsense-ros)

## Workflow
### Building Repository for the first time
1. `git clone https://github.com/williamx97/aqua_terra_rover.git`
2. `catkin build aqua_terra_rover`

### Getting latest files from github
1. `git pull`
2. `catkin build aqua_terra_rover`

## Demos
### Point to Point Navigation
`roslaunch aqua_terra_rover grid_move.launch`

To publish a point to the algorithm (run in a seperate terminal)

`rostopic pub /goalPose geometry_msgs/Pose2D "x: 0.0
y: 0.0
theta: 0.0" `


### ROS Navigation Stack (Incomplete)
`roslaunch aqua_terra_rover navigation.launch`
### 2D Occupancy Mapping 
`roslaunch aqua_terra_rover occupancy_live_custom.launch`
### Moving Rover through ROS via keyboard
`roslaunch aqua_terra_rover occupancy_live_custom.launch`

Then in a seperate terminal
`rosrun tr_teleop tr_teleop_key`

## Known Issues
* Cannot run Turtle Rover ROS node tr_control and the Web Interface at the same time. Web interface will result in motor command being glitchy but controlling motors via ROS will be fine
* Do not run ```sudo apt-get upgrade``` or ```sudo apt-get dist-upgrade``` This will break the ability to connect to the rover via TurtleRover Wifi. You will need to install the dependencies of any packages manually. 
* The D435 camera frequently does not get detected properly due to an issue mentioned here: https://github.com/IntelRealSense/librealsense/issues/8874 . D435 must be detected correctly through `rs-enumerate-devices` prior to running roslaunch commands. If D435 not detected reverse USB-C side of USB cable. If still not detected run `sudo reboot` 
* Crashes can frequently occur due to the low processing power of the Raspberry Pi

