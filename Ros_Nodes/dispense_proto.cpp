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

// TF
//#include <moveit/robot_state/joint_state_group.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <chef_arm_kinematics/ik_utils_proto.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <sstream>
//#include "fived_moveit/dispense.h"

using namespace std;

int main(int argc, char *argv[])
{
  std::vector<double> solution ;
  bool b ;
  initializeLimits() ;
    float yaw;
  
  ros::init (argc, argv, "blabla");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroup group("chef_arm");
  moveit_msgs::CollisionObject box;
  moveit_msgs::AttachedCollisionObject abox;
  moveit_msgs::DisplayTrajectory display_trajectory;

  geometry_msgs::Pose pose2;
  geometry_msgs::Pose target2;
  geometry_msgs::Pose target3;
  //-------------------------Move in the center of Pan----------------------------
  pose2.position.x = 0.0970104 ;
  pose2.position.y = -0.213201 ;
  pose2.position.z = 0.140673 ;
  /*
  pose2.orientation.x = 0 ;
  pose2.orientation.y = 0 ;
  pose2.orientation.z = -0.541221 ;
  pose2.orientation.w = 0.840881 ;
  */
  yaw = atan2(pose2.position.y,pose2.position.x);
  pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0.0,yaw) ;

  solution = checkIK(pose2,b) ;
  ROS_INFO_STREAM("Joint1: " << solution[0]) ;
  ROS_INFO_STREAM("Joint2: " << solution[1]) ;
  ROS_INFO_STREAM("Joint3: " << solution[2]) ;
  ROS_INFO_STREAM("Joint4: " << solution[3]) ;
  ROS_INFO_STREAM("Joint5: " << solution[4]) ;
  ROS_INFO_STREAM("Joint6: " << solution[5]) ; 

  if (b)
  {
     ROS_INFO_STREAM("Target is REACHABLE") ;
     group.setJointValueTarget(solution) ;
     group.move() ;
  }
  else
  {
     ROS_INFO_STREAM("Target is OUT OF REACH") ;
  }
  int dispense_quant = atoi(argv[1]);
  for (int i = 1; i<= dispense_quant;i++)
  { 	
 // ----------------------- First 90-------------------------
  pose2.position.x = 0.109651 ;
  pose2.position.y = -0.22642 ;
  pose2.position.z = 0.139725 ;

  /*
  pose2.orientation.x = 0 ;
  pose2.orientation.y = 0 ;
  pose2.orientation.z = -0.541221 ;
  pose2.orientation.w = 0.840881 ;
  */
  yaw = atan2(pose2.position.y,pose2.position.x);
  pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57,0.0,yaw) ;

  solution = checkIK(pose2,b) ;
  ROS_INFO_STREAM("Joint1: " << solution[0]) ;
  ROS_INFO_STREAM("Joint2: " << solution[1]) ;
  ROS_INFO_STREAM("Joint3: " << solution[2]) ;
  ROS_INFO_STREAM("Joint4: " << solution[3]) ;
  ROS_INFO_STREAM("Joint5: " << solution[4]) ;
  ROS_INFO_STREAM("Joint6: " << solution[5]) ; 

  if (b)
  {
     ROS_INFO_STREAM("Target is REACHABLE") ;
     group.setJointValueTarget(solution) ;
     group.move() ;
  }
  else
  {
     ROS_INFO_STREAM("Target is OUT OF REACH") ;
  }
  //sleep(1.0);
 // ----------------------- Second 90-------------------------
  pose2.position.x = 0.109651 ;
  pose2.position.y = -0.22642 ;
  pose2.position.z = 0.139725 ;

  /*
  pose2.orientation.x = 0 ;
  pose2.orientation.y = 0 ;
  pose2.orientation.z = -0.541221 ;
  pose2.orientation.w = 0.840881 ;
  */
  yaw = atan2(pose2.position.y,pose2.position.x);
  pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,0.0,yaw) ;


  solution = checkIK(pose2,b) ;
  ROS_INFO_STREAM("Joint1: " << solution[0]) ;
  ROS_INFO_STREAM("Joint2: " << solution[1]) ;
  ROS_INFO_STREAM("Joint3: " << solution[2]) ;
  ROS_INFO_STREAM("Joint4: " << solution[3]) ;
  ROS_INFO_STREAM("Joint5: " << solution[4]) ;
  ROS_INFO_STREAM("Joint6: " << solution[5]) ; 

  if (b)
  {
     ROS_INFO_STREAM("Target is REACHABLE") ;
     group.setJointValueTarget(solution) ;
     group.move() ;
  }
  else
  {
     ROS_INFO_STREAM("Target is OUT OF REACH") ;
  }
  //sleep(1.0);
 // ----------------------- Third 90-------------------------

  pose2.position.x = 0.109651 ;
  pose2.position.y = -0.22642 ;
  pose2.position.z = 0.139725 ;
  /*
  pose2.orientation.x = 0 ;
  pose2.orientation.y = 0 ;
  pose2.orientation.z = -0.541221 ;
  pose2.orientation.w = 0.840881 ;
  */
  yaw = atan2(pose2.position.y,pose2.position.x);
  pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(-1.57,0.0,yaw) ;

  solution = checkIK(pose2,b) ;
  ROS_INFO_STREAM("Joint1: " << solution[0]) ;
  ROS_INFO_STREAM("Joint2: " << solution[1]) ;
  ROS_INFO_STREAM("Joint3: " << solution[2]) ;
  ROS_INFO_STREAM("Joint4: " << solution[3]) ;
  ROS_INFO_STREAM("Joint5: " << solution[4]) ;
  ROS_INFO_STREAM("Joint6: " << solution[5]) ; 

  if (b)
  {
     ROS_INFO_STREAM("Target is REACHABLE") ;
     group.setJointValueTarget(solution) ;
     group.move() ;
  }
  else
  {
     ROS_INFO_STREAM("Target is OUT OF REACH") ;
  }
   // sleep(1.0);
 // ----------------------- Fourth 90-------------------------
  pose2.position.x = 0.109651 ;
  pose2.position.y = -0.22642 ;
  pose2.position.z = 0.139725 ;

  /*
  pose2.orientation.x = 0 ;
  pose2.orientation.y = 0 ;
  pose2.orientation.z = -0.541221 ;
  pose2.orientation.w = 0.840881 ;
  */
  yaw = atan2(pose2.position.y,pose2.position.x);
  pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(-3.14,0.0,yaw) ;

  solution = checkIK(pose2,b) ;
  ROS_INFO_STREAM("Joint1: " << solution[0]) ;
  ROS_INFO_STREAM("Joint2: " << solution[1]) ;
  ROS_INFO_STREAM("Joint3: " << solution[2]) ;
  ROS_INFO_STREAM("Joint4: " << solution[3]) ;
  ROS_INFO_STREAM("Joint5: " << solution[4]) ;
  ROS_INFO_STREAM("Joint6: " << solution[5]) ; 

  if (b)
  {
     ROS_INFO_STREAM("Target is REACHABLE") ;
     group.setJointValueTarget(solution) ;
     group.move() ;
  }
  else
  {
     ROS_INFO_STREAM("Target is OUT OF REACH") ;
  }
 // ----------------------- Back to 0-------------------------
  pose2.position.x = 0.109651 ;
  pose2.position.y = -0.22642 ;
  pose2.position.z = 0.139725 ;

  /*
  pose2.orientation.x = 0 ;
  pose2.orientation.y = 0 ;
  pose2.orientation.z = -0.541221 ;
  pose2.orientation.w = 0.840881 ;
  */
  yaw = atan2(pose2.position.y,pose2.position.x);
  pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0.0,yaw) ;

  solution = checkIK(pose2,b) ;
  ROS_INFO_STREAM("Joint1: " << solution[0]) ;
  ROS_INFO_STREAM("Joint2: " << solution[1]) ;
  ROS_INFO_STREAM("Joint3: " << solution[2]) ;
  ROS_INFO_STREAM("Joint4: " << solution[3]) ;
  ROS_INFO_STREAM("Joint5: " << solution[4]) ;
  ROS_INFO_STREAM("Joint6: " << solution[5]) ;  

  if (b)
  {
     ROS_INFO_STREAM("Target is REACHABLE") ;
     group.setJointValueTarget(solution) ;
     group.move() ;
  }
  else
  {
     ROS_INFO_STREAM("Target is OUT OF REACH") ;
  }  
  //sleep(1.0);
}
}

void dispenseit(int num)
{
  std::vector<double> solution ;
  bool b ;
  initializeLimits() ;
    float yaw;
  
  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroup group("chef_arm");
  moveit_msgs::CollisionObject box;
  moveit_msgs::AttachedCollisionObject abox;
  moveit_msgs::DisplayTrajectory display_trajectory;

  geometry_msgs::Pose pose2;
  geometry_msgs::Pose target2;
  geometry_msgs::Pose target3;
  //-------------------------Move in the center of Pan----------------------------
  pose2.position.x = 0.0970104 ;
  pose2.position.y = -0.213201 ;
  pose2.position.z = 0.140673 ;
  /*
  pose2.orientation.x = 0 ;
  pose2.orientation.y = 0 ;
  pose2.orientation.z = -0.541221 ;
  pose2.orientation.w = 0.840881 ;
  */
  yaw = atan2(pose2.position.y,pose2.position.x);
  pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0.0,yaw) ;

  solution = checkIK(pose2,b) ;
  ROS_INFO_STREAM("Joint1: " << solution[0]) ;
  ROS_INFO_STREAM("Joint2: " << solution[1]) ;
  ROS_INFO_STREAM("Joint3: " << solution[2]) ;
  ROS_INFO_STREAM("Joint4: " << solution[3]) ;
  ROS_INFO_STREAM("Joint5: " << solution[4]) ;
  ROS_INFO_STREAM("Joint6: " << solution[5]) ; 

  if (b)
  {
     ROS_INFO_STREAM("Target is REACHABLE") ;
     group.setJointValueTarget(solution) ;
     group.move() ;
  }
  else
  {
     ROS_INFO_STREAM("Target is OUT OF REACH") ;
  }
  int dispense_quant = num;
  for (int i = 1; i<= dispense_quant;i++)
  {   
 // ----------------------- First 90-------------------------
  pose2.position.x = 0.109651 ;
  pose2.position.y = -0.22642 ;
  pose2.position.z = 0.139725 ;

  /*
  pose2.orientation.x = 0 ;
  pose2.orientation.y = 0 ;
  pose2.orientation.z = -0.541221 ;
  pose2.orientation.w = 0.840881 ;
  */
  yaw = atan2(pose2.position.y,pose2.position.x);
  pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57,0.0,yaw) ;

  solution = checkIK(pose2,b) ;
  ROS_INFO_STREAM("Joint1: " << solution[0]) ;
  ROS_INFO_STREAM("Joint2: " << solution[1]) ;
  ROS_INFO_STREAM("Joint3: " << solution[2]) ;
  ROS_INFO_STREAM("Joint4: " << solution[3]) ;
  ROS_INFO_STREAM("Joint5: " << solution[4]) ;
  ROS_INFO_STREAM("Joint6: " << solution[5]) ; 

  if (b)
  {
     ROS_INFO_STREAM("Target is REACHABLE") ;
     group.setJointValueTarget(solution) ;
     group.move() ;
  }
  else
  {
     ROS_INFO_STREAM("Target is OUT OF REACH") ;
  }
  //sleep(1.0);
 // ----------------------- Second 90-------------------------
  pose2.position.x = 0.109651 ;
  pose2.position.y = -0.22642 ;
  pose2.position.z = 0.139725 ;

  /*
  pose2.orientation.x = 0 ;
  pose2.orientation.y = 0 ;
  pose2.orientation.z = -0.541221 ;
  pose2.orientation.w = 0.840881 ;
  */
  yaw = atan2(pose2.position.y,pose2.position.x);
  pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14,0.0,yaw) ;


  solution = checkIK(pose2,b) ;
  ROS_INFO_STREAM("Joint1: " << solution[0]) ;
  ROS_INFO_STREAM("Joint2: " << solution[1]) ;
  ROS_INFO_STREAM("Joint3: " << solution[2]) ;
  ROS_INFO_STREAM("Joint4: " << solution[3]) ;
  ROS_INFO_STREAM("Joint5: " << solution[4]) ;
  ROS_INFO_STREAM("Joint6: " << solution[5]) ; 

  if (b)
  {
     ROS_INFO_STREAM("Target is REACHABLE") ;
     group.setJointValueTarget(solution) ;
     group.move() ;
  }
  else
  {
     ROS_INFO_STREAM("Target is OUT OF REACH") ;
  }
  //sleep(1.0);
 // ----------------------- Third 90-------------------------

  pose2.position.x = 0.109651 ;
  pose2.position.y = -0.22642 ;
  pose2.position.z = 0.139725 ;
  /*
  pose2.orientation.x = 0 ;
  pose2.orientation.y = 0 ;
  pose2.orientation.z = -0.541221 ;
  pose2.orientation.w = 0.840881 ;
  */
  yaw = atan2(pose2.position.y,pose2.position.x);
  pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(-1.57,0.0,yaw) ;

  solution = checkIK(pose2,b) ;
  ROS_INFO_STREAM("Joint1: " << solution[0]) ;
  ROS_INFO_STREAM("Joint2: " << solution[1]) ;
  ROS_INFO_STREAM("Joint3: " << solution[2]) ;
  ROS_INFO_STREAM("Joint4: " << solution[3]) ;
  ROS_INFO_STREAM("Joint5: " << solution[4]) ;
  ROS_INFO_STREAM("Joint6: " << solution[5]) ; 

  if (b)
  {
     ROS_INFO_STREAM("Target is REACHABLE") ;
     group.setJointValueTarget(solution) ;
     group.move() ;
  }
  else
  {
     ROS_INFO_STREAM("Target is OUT OF REACH") ;
  }
   // sleep(1.0);
 // ----------------------- Fourth 90-------------------------
  pose2.position.x = 0.109651 ;
  pose2.position.y = -0.22642 ;
  pose2.position.z = 0.139725 ;

  /*
  pose2.orientation.x = 0 ;
  pose2.orientation.y = 0 ;
  pose2.orientation.z = -0.541221 ;
  pose2.orientation.w = 0.840881 ;
  */
  yaw = atan2(pose2.position.y,pose2.position.x);
  pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(-3.14,0.0,yaw) ;

  solution = checkIK(pose2,b) ;
  ROS_INFO_STREAM("Joint1: " << solution[0]) ;
  ROS_INFO_STREAM("Joint2: " << solution[1]) ;
  ROS_INFO_STREAM("Joint3: " << solution[2]) ;
  ROS_INFO_STREAM("Joint4: " << solution[3]) ;
  ROS_INFO_STREAM("Joint5: " << solution[4]) ;
  ROS_INFO_STREAM("Joint6: " << solution[5]) ; 

  if (b)
  {
     ROS_INFO_STREAM("Target is REACHABLE") ;
     group.setJointValueTarget(solution) ;
     group.move() ;
  }
  else
  {
     ROS_INFO_STREAM("Target is OUT OF REACH") ;
  }
 // ----------------------- Back to 0-------------------------
  pose2.position.x = 0.109651 ;
  pose2.position.y = -0.22642 ;
  pose2.position.z = 0.139725 ;

  /*
  pose2.orientation.x = 0 ;
  pose2.orientation.y = 0 ;
  pose2.orientation.z = -0.541221 ;
  pose2.orientation.w = 0.840881 ;
  */
  yaw = atan2(pose2.position.y,pose2.position.x);
  pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0.0,yaw) ;

  solution = checkIK(pose2,b) ;
  ROS_INFO_STREAM("Joint1: " << solution[0]) ;
  ROS_INFO_STREAM("Joint2: " << solution[1]) ;
  ROS_INFO_STREAM("Joint3: " << solution[2]) ;
  ROS_INFO_STREAM("Joint4: " << solution[3]) ;
  ROS_INFO_STREAM("Joint5: " << solution[4]) ;
  ROS_INFO_STREAM("Joint6: " << solution[5]) ;  

  if (b)
  {
     ROS_INFO_STREAM("Target is REACHABLE") ;
     group.setJointValueTarget(solution) ;
     group.move() ;
  }
  else
  {
     ROS_INFO_STREAM("Target is OUT OF REACH") ;
  }  
  //sleep(1.0);
}
}