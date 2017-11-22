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

using namespace std;
/*
int main(int argc, char *argv[])
{
  // Given -> x,y,z
  // Target1 -> x1,y1,z1
  // Target2 -> x2,y2,z2
  // Temporary -> Slope,Distance,yaw -> m,d,yaw
//---------------------------------------INITIALIZING SQL DATABASE-----------------------------------------------//
  MYSQL *connection = mysql_init(NULL);
  connection = mysql_real_connect(connection, "localhost", "root", "nymble", "nymble", 0, NULL, 0);
  MYSQL_RES *result;  
  MYSQL_ROW row;     
//------------------------------------------------------------Initializing Variables----------------------------------------------------------------//
  std::string arg = argv[1];  
  int num_of_fields = 0;  
  float x,y,z,r,p,yw;
  float x1,y1,z1;
  float x2,y2,z2;
  float m,d,yaw;
  std::vector<double> solution ;
  bool b ;
  initializeLimits() ;
  
  ros::init (argc, argv, "blabla");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroup group("chef_arm");
  moveit_msgs::CollisionObject box;
  moveit_msgs::AttachedCollisionObject abox;
  moveit_msgs::DisplayTrajectory display_trajectory;

  geometry_msgs::Pose target1;
  geometry_msgs::Pose target2;
  geometry_msgs::Pose target3;

  ros::NodeHandle nh;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

  ros::Duration sleep_time(10.0);
  ros::WallDuration(1.0).sleep();

  group.setPlanningTime(45.0);
//---------------------------------------------------------Retreiving Box Data-----------------------------------------------------------------//  
  std::string s = "SELECT * FROM objects where id = ";
  std::string str = s + arg;
  const char * c = str.c_str();  
  if(mysql_query(connection, c)) 
  {      
      ROS_INFO("Query Error: %s", mysql_error(connection));  
      exit(1);  
  }  
  else  
  {  
      result = mysql_store_result(connection); 
      num_of_fields = mysql_num_fields(result);
      row = mysql_fetch_row(result);
      x = atof(row[2]);
      y = atof(row[3]);
      z = atof(row[4]);
      r = atof(row[5]);
      p = atof(row[6]);
      yw = atof(row[7]);
  }
  ROS_INFO("FISRT VALUES ");
  ROS_INFO("Value x %f",x);
  ROS_INFO("Value y %f",y);
  ROS_INFO("Value z %f",z);

  z = z + 0.013756;

//---------------------------------------------------------Trajecotry Planner---------------------------------------------------------------------//
//-------------------------------------------------------------Target 1------------------------------------------------------------------------------//
// Distance from origin of given point
  d = sqrt((x*x) + (y*y));
  d = d - 0.1; 
// slope of line from origin
  if(x!=0)
  {
    m = y/x;
    if(x<0)
      x1 = -sqrt((d*d)/((m*m)+1));
    else
      x1 = sqrt((d*d)/((m*m)+1));
    y1 = m*x1;
  }
  else
  {
    x1 = x;
    y1 = y - 0.1;
  } 
  ROS_INFO("SECOND VALUES ");
  ROS_INFO("Value x %f",x1);
  ROS_INFO("Value y %f",y1);
  ROS_INFO("Value z %f",z);  

  yaw = atan2(y1,x1);
  target1.position.x = x1;
  target1.position.y = y1;
  target1.position.z = z;//

  target1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,yaw) ;

  solution = checkIK(target1,b) ;
  ROS_INFO_STREAM("Joint1: " << solution[0]) ;
  ROS_INFO_STREAM("Joint2: " << solution[1]) ;
  ROS_INFO_STREAM("Joint3: " << solution[2]) ;
  ROS_INFO_STREAM("Joint4: " << solution[3]) ;
  ROS_INFO_STREAM("Joint5: " << solution[4]) ;
  ROS_INFO_STREAM("Joint6: " << solution[5]) ;
  ROS_INFO_STREAM("B value : "<< b);

  ROS_INFO_STREAM("zdfhsdifhszdddddddddddddddddddddddddddddd");

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
/*  ROS_INFO("W -> %f",   target1.orientation.w);
  ROS_INFO("x -> %f",   target1.orientation.x);
  ROS_INFO("y -> %f",   target1.orientation.y);
  ROS_INFO("z -> %f",   target1.orientation.z);
*/

//---------------------------------------------------------------------Target 2----------------------------------------------------------------------//
// Distance from origin of given point
/*  d = sqrt((x*x) + (y*y));
  d = d - 0.05; 
// slope of line from origin
  if(x!=0)
  {
    m = y/x;
    if(x<0)
      x = -sqrt((d*d)/((m*m)+1));
    else
      x = sqrt((d*d)/((m*m)+1));
    y = m*x;
  }
  else
  {
    y = y - 0.06;
  } 
 
  ROS_INFO("THIRD VALUES ");
  ROS_INFO("Value x %f",x);
  ROS_INFO("Value y %f",y);
  ROS_INFO("Value z %f",z);
  target2.position.x = x;
  target2.position.y = y;
  target2.position.z = z;

  yaw = atan2(y,x);

  target2.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,yaw) ;
  
  solution = checkIK(target2,b) ;
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
//----------------------------------------------------Attach object to robot----------------------------------------------------//

  ROS_INFO("Attach the object to the robot");
  std::string st = "";
  st = "box" + arg;
  box.id = st;
  group.attachObject(box.id);

  sleep(2.0);

//------------------------------------------------------------------Random/Subsequent Targets---------------------------------------------//
  solution = checkIK(target1,b) ;
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
  ros::waitForShutdown();
  return 0;
  }
*/
//************************************************************************************************************************************************//
//*********************************FUNCTION FOR PICKING***************************************************//
//************************************************************************************************************************************************//

void pickit(int ind)
{
  ROS_INFO("ENTRY INTO PICKIT");
  // Given -> x,y,z
  // Target1 -> x1,y1,z1
  // Target2 -> x2,y2,z2
  // Temporary -> Slope,Distance,yaw -> m,d,yaw
//---------------------------------------INITIALIZING SQL DATABASE-----------------------------------------------//
  MYSQL *connection = mysql_init(NULL);
  connection = mysql_real_connect(connection, "localhost", "root", "nymble", "nymble", 0, NULL, 0);
  MYSQL_RES *result;  
  MYSQL_ROW row;     
//------------------------------------------------------------Initializing Variables----------------------------------------------------------------//
  stringstream ss;
  ss << ind;
  std::string arg = ss.str();  
  int num_of_fields = 0;  
  float x,y,z,r,p,yw;
  float x1,y1,z1;
  float x2,y2,z2;
  float m,d,yaw;
  std::vector<double> solution ;
  bool b ;
  initializeLimits() ;
  
  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroup group("chef_arm");
  moveit_msgs::CollisionObject box;
  moveit_msgs::AttachedCollisionObject abox;
  moveit_msgs::DisplayTrajectory display_trajectory;

  geometry_msgs::Pose target1;
  geometry_msgs::Pose target2;
  geometry_msgs::Pose target3;

  ros::NodeHandle nh;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

  ros::Duration sleep_time(10.0);
  ros::WallDuration(1.0).sleep();

  group.setPlanningTime(45.0);
//---------------------------------------------------------Retreiving Box Data-----------------------------------------------------------------//  
  std::string s = "SELECT * FROM objects where id = ";
  std::string str = s + arg;
  const char * c = str.c_str();  
  if(mysql_query(connection, c)) 
  {      
      ROS_INFO("Query Error: %s", mysql_error(connection));  
      exit(1);  
  }  
  else  
  {  
      result = mysql_store_result(connection); 
      num_of_fields = mysql_num_fields(result);
      row = mysql_fetch_row(result);
      x = atof(row[2]);
      y = atof(row[3]);
      z = atof(row[4]);
      r = atof(row[5]);
      p = atof(row[6]);
      yw = atof(row[7]);
  }
  ROS_INFO("FISRT VALUES ");
  ROS_INFO("Value x %f",x);
  ROS_INFO("Value y %f",y);
  ROS_INFO("Value z %f",z);

  z = z + 0.013756;

//---------------------------------------------------------Trajecotry Planner---------------------------------------------------------------------//
//-------------------------------------------------------------Target 1------------------------------------------------------------------------------//
// Distance from origin of given point
  d = sqrt((x*x) + (y*y));
  d = d - 0.1; 
// slope of line from origin
  if(x!=0)
  {
    m = y/x;
    if(x<0)
      x1 = -sqrt((d*d)/((m*m)+1));
    else
      x1 = sqrt((d*d)/((m*m)+1));
    y1 = m*x1;
  }
  else
  {
    x1 = x;
    y1 = y - 0.1;
  } 
  ROS_INFO("SECOND VALUES ");
  ROS_INFO("Value x %f",x1);
  ROS_INFO("Value y %f",y1);
  ROS_INFO("Value z %f",z);  

  yaw = atan2(y1,x1);
  target1.position.x = x1;
  target1.position.y = y1;
  target1.position.z = z;//

  target1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,yaw) ;

  solution = checkIK(target1,b) ;
  ROS_INFO_STREAM("Joint1: " << solution[0]) ;
  ROS_INFO_STREAM("Joint2: " << solution[1]) ;
  ROS_INFO_STREAM("Joint3: " << solution[2]) ;
  ROS_INFO_STREAM("Joint4: " << solution[3]) ;
  ROS_INFO_STREAM("Joint5: " << solution[4]) ;
  ROS_INFO_STREAM("Joint6: " << solution[5]) ;
  //ROS_INFO_STREAM("B value : "<< b);
  //ROS_INFO_STREAM("sdkfnsdfnskjfnksnekjsenfjkesf");
  if (b)
  {
     ROS_INFO_STREAM("Target is REACHABLE") ;
     //group.setJointValueTarget(solution) ;
    moveit::planning_interface::MoveGroup::Plan my_plan;
    group.setPoseTarget(target1) ;
    bool success = group.plan(my_plan) ;

  group.move() ;
  sleep(5.0) ;
  }
  else
  {
     ROS_INFO_STREAM("Target is OUT OF REACH") ;
  }
/*  ROS_INFO("W -> %f",   target1.orientation.w);
  ROS_INFO("x -> %f",   target1.orientation.x);
  ROS_INFO("y -> %f",   target1.orientation.y);
  ROS_INFO("z -> %f",   target1.orientation.z);
*/

//---------------------------------------------------------------------Target 2----------------------------------------------------------------------//
// Distance from origin of given point
  d = sqrt((x*x) + (y*y));
  d = d - 0.05; 
// slope of line from origin
  if(x!=0)
  {
    m = y/x;
    if(x<0)
      x = -sqrt((d*d)/((m*m)+1));
    else
      x = sqrt((d*d)/((m*m)+1));
    y = m*x;
  }
  else
  {
    y = y - 0.06;
  } 
 
  ROS_INFO("THIRD VALUES ");
  ROS_INFO("Value x %f",x);
  ROS_INFO("Value y %f",y);
  ROS_INFO("Value z %f",z);
  target2.position.x = x;
  target2.position.y = y;
  target2.position.z = z;

  yaw = atan2(y,x);

  target2.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,yaw) ;
  
  solution = checkIK(target2,b) ;
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
//----------------------------------------------------Attach object to robot----------------------------------------------------//

  ROS_INFO("Attach the object to the robot");
  std::string st = "";
  st = "box" + arg;
  box.id = st;
  group.attachObject(box.id);

  sleep(2.0);

//------------------------------------------------------------------Random/Subsequent Targets---------------------------------------------//
  solution = checkIK(target1,b) ;
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
    ROS_INFO("LEAVING PICKIT");
}











