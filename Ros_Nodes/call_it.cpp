#include "/home/mandeep/catkin_ws/src/proto1/proto1_moveit/include/proto1_moveit/pick_proto.h"

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <mysql/mysql.h>

// TF
//#include <moveit/robot_state/joint_state_group.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <chef_arm_kinematics/ik_utils_proto.h>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <mysql/mysql.h>

int main(int argc, char *argv[]){
	ros::init (argc, argv, "blabla");
	
	//pickit(1);
	return 0;
}
