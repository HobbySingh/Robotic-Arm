#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>

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
#include <string>
#include <mysql/mysql.h>

using namespace std;
//#define BN = 5;
int main(int argc, char *argv[])
{
ROS_INFO("INTO THE ENVIRONMENT_PROTO");  
//---------------------------------------INITIALIZING SQL DATABASE-----------------------------------------------//
  MYSQL *connection = mysql_init(NULL);
  connection = mysql_real_connect(connection, "localhost", "root", "nymble", "nymble", 0, NULL, 0);
  MYSQL_RES *result;  
  MYSQL_ROW row;
//-------------------------------------Initializing Variables--------------------------
int l ;
int spiceboxes = 11;
int num_of_rows;
double x,y,z,r,p,yaw;
std::string s;
std::string str;
stringstream ss;
std::string int2str;
char int2charp[100];
const char * c; 

ros::init (argc, argv, "blabla");
ros::AsyncSpinner spinner(2);
spinner.start();

moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
moveit::planning_interface::MoveGroup group("chef_arm");

std::vector<moveit_msgs::CollisionObject> collision_objects;
//--------------------------------------Updating no. of boxes--------------------------------------//
  if(mysql_query(connection, "SELECT * FROM objects")) 
  {      
      ROS_INFO("Query Error: %s", mysql_error(connection));  
      exit(1);  
  }  
  else  
  {  
      result = mysql_store_result(connection); 
      num_of_rows = mysql_num_rows(result);
      row = mysql_fetch_row(result);
      x = atof(row[2]);
      y = atof(row[3]);
      z = atof(row[4]);
      r = atof(row[5]);
      p = atof(row[6]);
      yaw = atof(row[7]);
  }  
  int BN = num_of_rows;
  ROS_INFO("No.of boxes %d", BN);

//-------------------------------------------------------------ADDING ENVIRONMENT STRUCTURE----------------------------------------//
  moveit_msgs::CollisionObject co2;
  co2.header.frame_id = group.getPlanningFrame();

  shapes::Mesh* m2 = shapes::createMeshFromResource("file:///home/mandeep/catkin_ws/src/proto1/proto1_moveit/meshes/Product Outline_ros1.STL");
  shape_msgs::Mesh co_mesh2;
  shapes::ShapeMsg co_mesh_msg2;
  shapes::constructMsgFromShape(m2,co_mesh_msg2);
  co_mesh2 = boost::get<shape_msgs::Mesh>(co_mesh_msg2);
  co2.meshes.resize(1);
  co2.meshes[0] = co_mesh2;
  co2.mesh_poses.resize(1);
  co2.mesh_poses[0].position.x = 0;
  co2.mesh_poses[0].position.y = 0;
  co2.mesh_poses[0].position.z = -0.015;
  co2.mesh_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw( 1.57, 0.0, 0.0 ) ;  
  co2.id = "productstructure" ;

  co2.meshes.push_back(co_mesh2);
  co2.mesh_poses.push_back(co2.mesh_poses[0]);
  co2.operation = co2.ADD;

  collision_objects.push_back(co2);

  ROS_INFO("Add Product Structure to world");
 planning_scene_interface.addCollisionObjects(collision_objects);

//-------------------------------------------------------------ADDING BOXES-----------------------------------------------------------
  moveit_msgs::CollisionObject co;
  shapes::Mesh* m = shapes::createMeshFromResource("file:///home/mandeep/catkin_ws/src/proto1/proto1_moveit/meshes/Spice Container.STL");
  shape_msgs::Mesh co_mesh;
  shapes::ShapeMsg co_mesh_msg;
  shapes::constructMsgFromShape(m,co_mesh_msg);
  co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

for(int i = 1; i <= spiceboxes; i++)
{

  ss.str(std::string());
  ss << i;
  int2str = ss.str();
  s = "SELECT * FROM objects where id = ";
  str = s + int2str;
    ROS_INFO_STREAM("String for sql" +  str);
   c = str.c_str();  
  if(mysql_query(connection, c)) 
  {      
      ROS_INFO("Query Error: %s", mysql_error(connection));  
      exit(1);  
  }  
  else  
  {  
      result = mysql_store_result(connection); 
      row = mysql_fetch_row(result);
      x = atof(row[2]);
      y = atof(row[3]);
      z = atof(row[4]);
      r = atof(row[5]);
      p = atof(row[6]);
      yaw = atof(row[7]);
          ROS_INFO("Value x %f y %f z %f r %f  p %f yaw %f", x,y,z,r,p,yaw);
  }
  co.header.frame_id = group.getPlanningFrame();

  co.meshes.resize(1);
  co.meshes[0] = co_mesh;
  co.mesh_poses.resize(1);

  co.mesh_poses[0].position.x = x;
  co.mesh_poses[0].position.y = y;
  co.mesh_poses[0].position.z = z+0.0025;
  co.mesh_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(r , p, yaw) ;  
  

  co.id = "box" + int2str; 
  std::cout << co.id <<'\n'; 

  co.meshes.push_back(co_mesh);   
    ROS_INFO("CHECK1");  
  co.mesh_poses.push_back(co.mesh_poses[0]);
  co.operation = co.ADD;
  ROS_INFO("CHECK2");  
  collision_objects.push_back(co);

  ROS_INFO("Add Box Structure to world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  co.meshes.clear();
   
  co.mesh_poses.clear();

  /* Sleep so we have time to see the object in RViz */
}
  m = shapes::createMeshFromResource("file:///home/mandeep/catkin_ws/src/proto1/proto1_moveit/meshes/veg_box.STL");
  shapes::constructMsgFromShape(m,co_mesh_msg);
  co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

for(int i = spiceboxes+1; i <= BN; i++)
{

  ss.str(std::string());
  ss << i;
  int2str = ss.str();
  s = "SELECT * FROM objects where id = ";
  str = s + int2str;
    ROS_INFO_STREAM("String for sql" +  str);
   c = str.c_str();  
  if(mysql_query(connection, c)) 
  {      
      ROS_INFO("Query Error: %s", mysql_error(connection));  
      exit(1);  
  }  
  else  
  {  
      result = mysql_store_result(connection); 
      row = mysql_fetch_row(result);
      x = atof(row[2]);
      y = atof(row[3]);
      z = atof(row[4]);
      r = atof(row[5]);
      p = atof(row[6]);
      yaw = atof(row[7]);
          ROS_INFO("Value x %f y %f z %f r %f  p %f yaw %f", x,y,z,r,p,yaw);
  }
  co.header.frame_id = group.getPlanningFrame();

  co.meshes.resize(1);
  co.meshes[0] = co_mesh;
  co.mesh_poses.resize(1);

  co.mesh_poses[0].position.x = x;
  co.mesh_poses[0].position.y = y;
  co.mesh_poses[0].position.z = z;
  co.mesh_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(r , p, yaw) ;  
  

  co.id = "box" + int2str; 
  std::cout << co.id <<'\n'; 

  co.meshes.push_back(co_mesh);   
    ROS_INFO("CHECK1");  
  co.mesh_poses.push_back(co.mesh_poses[0]);
  co.operation = co.ADD;
  ROS_INFO("CHECK2");  
  collision_objects.push_back(co);

  ROS_INFO("Add Box Structure to world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  co.meshes.clear();
   
  co.mesh_poses.clear();

  /* Sleep so we have time to see the object in RViz */
}
  ros::waitForShutdown();
  return 0;
}

