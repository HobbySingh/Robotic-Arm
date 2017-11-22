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
#include <moveit_msgs/RobotState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

// TF
#include <chef_arm_kinematics/ik_utils_proto.h>

#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "proto1_motion");
  ros::AsyncSpinner spinner(3);
  spinner.start();
  bool b ;
  std::vector<double> solution ;
  initializeLimits() ;

  // We will use the :planning_scene_interface:`PlanningSceneInterface
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  std::stringstream sstr ;

  ros::NodeHandle nh;

  moveit::planning_interface::MoveGroup group("chef_arm");
  group.setPlanningTime(20.0);

  // wait a bit for ros things to initialize
  ros::WallDuration(4.0).sleep();

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;


  std::vector<std::string> active_joints ;

  active_joints = group.getJoints() ;

  for ( int i = 0; i < active_joints.size(); i = i + 1 )
  {  
      sstr << active_joints[i] << std::endl ;
  }

  sstr << group.getGoalJointTolerance() << std::endl ;

  std::cout << sstr.str() << std::endl ;
  
  geometry_msgs::Pose target_pose ;
/*
  target_pose.position.x = 0.400148 ;
  target_pose.position.y = 0.123781 ;
  target_pose.position.z = 0.362729 ;

  target_pose.orientation.x = 0.0 ;
  target_pose.orientation.y = 0.0 ;
  target_pose.orientation.z = 0.149439; 
  target_pose.orientation.w = 0.988771 ;
*/
  target_pose.position.x = 0.00488 ;
  target_pose.position.y = 0.3143 ;
  target_pose.position.z = 0.02999 ;

  target_pose.orientation.x = 0 ;
  target_pose.orientation.y = 0 ;
  target_pose.orientation.z = 0.701592; 
  target_pose.orientation.w = 0.712579 ;

  solution = checkIK(target_pose,b) ;

/*  moveit::planning_interface::MoveGroup::Plan my_plan;
  group.setPoseTarget(target_pose) ;
  bool success = group.plan(my_plan) ;*/
  group.setJointValueTarget(solution) ;
  group.move() ;
  sleep(5.0) ;



  //*********************** Compute IK ***************************************//
  /**
  // Fill in Robot State
    moveit_msgs::RobotState r_state ;
    r_state.joint_state.name.resize(6) ;
    r_state.joint_state.name[0] = "joint1" ;
    r_state.joint_state.name[1] = "joint2" ;
    r_state.joint_state.name[2] = "joint2_2" ;
    r_state.joint_state.name[3] = "joint3" ;
    r_state.joint_state.name[4] = "joint3_2" ;
    r_state.joint_state.name[5] = "joint4" ;

    ros::ServiceClient fk_client = nh.serviceClient<moveit_msgs::GetPositionFK>("compute_fk") ;
    ros::ServiceClient ik_client = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik") ;

    moveit_msgs::GetPositionIK ik_msg ;

    // Fill in group name
    ik_msg.request.ik_request.group_name = "chef_arm" ;

    // Fill in Robot State
    r_state.joint_state.position.resize(6) ;
    r_state.joint_state.position[0] = 0.0 ;
    r_state.joint_state.position[1] = 0.0 ;
    r_state.joint_state.position[2] = 0.0 ;
    r_state.joint_state.position[3] = 0.0 ;
    r_state.joint_state.position[4] = 0.0 ;
    r_state.joint_state.position[5] = 0.0 ;

    ik_msg.request.ik_request.robot_state = r_state ;

    // Fill in ik_link_name
    ik_msg.request.ik_request.ik_link_name = "motor" ;

    // Fill in Pose
    geometry_msgs::PoseStamped target_pose ;
    target_pose.pose.position.x =  0.400148 ;
    target_pose.pose.position.y =  0.123781  ;
    target_pose.pose.position.z =  0.362729  ;

    target_pose.pose.orientation.x = 0.0 ;
    target_pose.pose.orientation.y = 0.0  ;
    target_pose.pose.orientation.z = 0.149439 ;
    target_pose.pose.orientation.w = 0.988771 ;

    ik_msg.request.ik_request.pose_stamped = target_pose ;

    // Call service
    ik_client.call(ik_msg) ;

    ss << std::fixed << "Joint1: " << ik_msg.response.solution.joint_state.position[0] << "\n ";
    ss << std::fixed << "Joint2: " << ik_msg.response.solution.joint_state.position[1] << "\n ";
    ss << std::fixed << "Joint3: " << ik_msg.response.solution.joint_state.position[2] << "\n ";
    ss << std::fixed << "Joint4: " << ik_msg.response.solution.joint_state.position[3] << "\n ";
    ss << std::fixed << "Joint5: " << ik_msg.response.solution.joint_state.position[4] << "\n ";
    ss << std::fixed << "Joint6: " << ik_msg.response.solution.joint_state.position[5] << "\n ";

    ROS_INFO_STREAM( ss.str() ) ;

  **/
  ros::waitForShutdown();
  return 0;
}
