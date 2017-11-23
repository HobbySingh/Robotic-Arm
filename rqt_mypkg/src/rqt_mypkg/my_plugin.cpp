#include "rqt_mypkg/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
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
#include <math.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <tf/transform_datatypes.h>
#include <mysql/mysql.h>

#include <chef_arm_kinematics/ik_utils_proto.h>
#include <proto1_moveit/pick_proto.h>
#include <proto1_moveit/place_proto.h>
#include <proto1_moveit/dispense_proto.h>
#include <proto1_moveit/stir_proto.h>

//#include <five_dof_arm_kinematics/ik_utils.h>
//#include <fived_moveit/pick.h>
//#include <fived_moveit/place.h>
//#include <fived_moveit/dispense.h>
//#include <fived_moveit/mandy_stir.h>

namespace rqt_mypkg{

// TF
//#include <moveit/robot_state/joint_state_group.h>
using namespace std;

/**
 * @brief MyPlugin::MyPlugin
 */
MyPlugin::MyPlugin()
    : rqt_gui_cpp::Plugin()
    , widget_(0) {
    
    setObjectName("MyPlugin");
}

/**
 * @brief MyPlugin::initPlugin
 * @param context
 */
void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
    widget_ = new QWidget();
    ui_.setupUi(widget_);
    if (context.serialNumber() > 1) {
        widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }
    context.addWidget(widget_);

    QSlider::TickPosition t = QSlider::TicksBelow;
    int degree_interval = 10;
    double single_step = 0.05;
    double single_step2 = 0.005;
// x slider
    ui_.x_slider->setRange(-45,45);
    ui_.x_slider->setTickPosition(t);
    ui_.x_slider->setTickInterval(1);
    ui_.x_slider->setValue(28);
    ui_.x_slider->setPageStep(1);
// y slider
    ui_.y_slider->setRange(-45,45);
    ui_.y_slider->setTickPosition(t);
    ui_.y_slider->setTickInterval(1);
    ui_.y_slider->setValue(0);
    ui_.y_slider->setPageStep(1);
// z slider
    ui_.z_slider->setRange(0.06,55);
    ui_.z_slider->setTickPosition(t);
    ui_.z_slider->setTickInterval(1);
    ui_.z_slider->setValue(36);
    ui_.z_slider->setPageStep(1);
// r slider
    ui_.r_slider->setRange(-314,314);
    ui_.r_slider->setTickPosition(t);
    ui_.r_slider->setTickInterval(1);// round about five degree change
    ui_.r_slider->setValue(0);
    ui_.r_slider->setPageStep(1);
// p slider
    ui_.p_slider->setRange(-157,157);
    ui_.p_slider->setTickPosition(t);
    ui_.p_slider->setTickInterval(1);
    ui_.p_slider->setValue(0);
    ui_.p_slider->setPageStep(1);
// yaw slider
    ui_.yaw_slider->setRange(-314,314);
    ui_.yaw_slider->setTickPosition(t);
    ui_.yaw_slider->setTickInterval(1);
    ui_.yaw_slider->setValue(0);
    ui_.yaw_slider->setPageStep(1);
// x spinbox
    ui_.x_spin->setRange(-0.45,0.45);
    ui_.x_spin->setValue(0.28);
    ui_.x_spin->setSingleStep(single_step2);
    ui_.x_spin->setKeyboardTracking(false);
// y spinbox
    ui_.y_spin->setRange(-0.45,0.45);
    ui_.y_spin->setValue(0);
    ui_.y_spin->setSingleStep(single_step2);
    ui_.y_spin->setKeyboardTracking(false);
// z spinbox
    ui_.z_spin->setRange(0,0.55);
    ui_.z_spin->setValue(0.36);
    ui_.z_spin->setSingleStep(single_step2);
    ui_.z_spin->setKeyboardTracking(false);
// r spinbox
    ui_.r_spin->setRange(-3.14,3.14);
    ui_.r_spin->setValue(0);
    ui_.r_spin->setSingleStep(single_step2);
    ui_.r_spin->setKeyboardTracking(false);
// p spinbox
    ui_.p_spin->setRange(-1.57,1.57);
    ui_.p_spin->setValue(0);
    ui_.p_spin->setSingleStep(single_step2);
    ui_.p_spin->setKeyboardTracking(false);
// yaw spinbox
    ui_.yaw_spin->setRange(-0.45,0.45);
    ui_.yaw_spin->setValue(0);
    ui_.yaw_spin->setSingleStep(single_step2);
    ui_.yaw_spin->setKeyboardTracking(false);
// j1 slider
    ui_.j1_slider->setRange(-314,314);
    ui_.j1_slider->setTickPosition(t);
    ui_.j1_slider->setTickInterval(degree_interval);
    ui_.j1_slider->setValue(0);
    ui_.j1_slider->setPageStep(degree_interval);
    ui_.j1_slider->setTracking(true);
//j2 slider
    ui_.j2_slider->setRange(0,314);
    ui_.j2_slider->setTickPosition(t);
    ui_.j2_slider->setTickInterval(degree_interval);
    ui_.j2_slider->setValue(157);
    ui_.j2_slider->setPageStep(degree_interval);
// j3 slider
    ui_.j3_slider->setRange(-200,200);
    ui_.j3_slider->setTickPosition(t);
    ui_.j3_slider->setTickInterval(degree_interval);
    ui_.j3_slider->setValue(0);
    ui_.j3_slider->setPageStep(degree_interval);
// j4 slider
    ui_.j4_slider->setRange(-314,314);
    ui_.j4_slider->setTickPosition(t);
    ui_.j4_slider->setTickInterval(degree_interval);
    ui_.j4_slider->setValue(0);
    ui_.j4_slider->setPageStep(degree_interval);
// j1 spin box
    ui_.j1_spin->setRange(-3.14,3.14);
    ui_.j1_spin->setValue(0);
    ui_.j1_spin->setSingleStep(single_step);
    ui_.j1_spin->setKeyboardTracking(false);
// j2 spin box
    ui_.j2_spin->setRange(0,3.14);
    ui_.j2_spin->setValue(1.57);
    ui_.j2_spin->setSingleStep(single_step);
    ui_.j2_spin->setKeyboardTracking(false);   
// j3 spin box
    ui_.j3_spin->setRange(-2.00,2.00);
    ui_.j3_spin->setValue(0);
    ui_.j3_spin->setSingleStep(single_step);  
    ui_.j3_spin->setKeyboardTracking(false);      
// j4 spin box
    ui_.j4_spin->setRange(-3.14,3.14);
    ui_.j4_spin->setValue(0);
    ui_.j4_spin->setSingleStep(single_step);
    ui_.j4_spin->setKeyboardTracking(false);             

    ui_.redbut -> setStyleSheet("background-color: rgb(255, 0, 0);");
    ui_.greenbut -> setStyleSheet(""); 
            
 //   
    //moveit::planning_interface::MoveGroup group("five_dof_arm");
    moveit::planning_interface::MoveGroup group("chef_arm");

    setupROSComponents();
    connectROSComponents();

}

void MyPlugin::shutdownPlugin() {
}

/**
 * @brief MyPlugin::saveSettings
 * @param plugin_settings
 * @param instance_settings
 */
void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                               qt_gui_cpp::Settings& instance_settings) const {
}

/**
 * @brief MyPlugin::restoreSettings
 * @param plugin_settings
 * @param instance_settings
 */
void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                  const qt_gui_cpp::Settings& instance_settings) {
}
void MyPlugin::connectROSComponents()
{
    connect(ui_.pick, SIGNAL(clicked()), this, SLOT(pick()));
    connect(ui_.place, SIGNAL(clicked()), this, SLOT(place()));
    connect(ui_.dispense, SIGNAL(clicked()), this, SLOT(dispense()));   
    connect(ui_.stir, SIGNAL(clicked()), this, SLOT(stir()));
    connect(ui_.home, SIGNAL(clicked()), this, SLOT(home()));
    connect(ui_.j1_slider, SIGNAL(sliderReleased()), this, SLOT(jointChange()));
    connect(ui_.j2_slider, SIGNAL(sliderReleased()), this, SLOT(jointChange()));
    connect(ui_.j3_slider, SIGNAL(sliderReleased()), this, SLOT(jointChange()));
    connect(ui_.j4_slider, SIGNAL(sliderReleased()), this, SLOT(jointChange()));
    connect(ui_.j1_spin, SIGNAL(valueChanged(double)), this, SLOT(jointChange2()));
    connect(ui_.j2_spin, SIGNAL(valueChanged(double)), this, SLOT(jointChange2()));
    connect(ui_.j3_spin, SIGNAL(valueChanged(double)), this, SLOT(jointChange2()));
    connect(ui_.j4_spin, SIGNAL(valueChanged(double)), this, SLOT(jointChange2()));

    connect(ui_.x_slider, SIGNAL(sliderReleased()), this, SLOT(positionChange()));
    connect(ui_.y_slider, SIGNAL(sliderReleased()), this, SLOT(positionChange()));    
    connect(ui_.z_slider, SIGNAL(sliderReleased()), this, SLOT(positionChange()));
    connect(ui_.r_slider, SIGNAL(sliderReleased()), this, SLOT(positionChange()));
    connect(ui_.p_slider, SIGNAL(sliderReleased()), this, SLOT(positionChange()));
    connect(ui_.yaw_slider, SIGNAL(sliderReleased()), this, SLOT(yawChange()));  
  
    connect(ui_.x_spin, SIGNAL(valueChanged(double)), this, SLOT(positionChange2()));
    connect(ui_.y_spin, SIGNAL(valueChanged(double)), this, SLOT(positionChange2()));
    connect(ui_.z_spin, SIGNAL(valueChanged(double)), this, SLOT(positionChange2()));
    connect(ui_.r_spin, SIGNAL(valueChanged(double)), this, SLOT(positionChange2()));
    connect(ui_.p_spin, SIGNAL(valueChanged(double)), this, SLOT(positionChange2()));
    connect(ui_.yaw_spin, SIGNAL(valueChanged(double)), this, SLOT(yawChange2()));

    connect(ui_.vacuum, SIGNAL(clicked()), this, SLOT(vacuum()));
}

void MyPlugin::temperature(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Plugin temperature");
    //-------For publishing on Topic---------
    /*double secs;
    std_msgs::Header h1;
    //-----------------------------------------Temperature Publishing on Temperature Topic-----------------------------------
    ros::NodeHandle nh;
    ros::Publisher temp_pub = nh.advertise<std_msgs::Header>("temp_proto",1000);
    
    ros::Duration sleep_time(10.0);
    ros::WallDuration(1.0).sleep();

    int temp = atoi(msg->data.c_str());
    ROS_INFO("Value of temp %d",temp);
    double present;
    double past = ros::Time::now().toSec();

    //--------------------Header that will be published to topic for rosbag------------------------
    secs = ros::Time::now().toSec();
    h1.frame_id = "Temperature";
    h1.stamp.sec = secs;
    h1.seq = (unsigned int)temp;
    temp_pub.publish(h1);   */ 
}
void MyPlugin::pick()
{
    QString err1 = "Status : Please select what to pick";
    QString err2 = "Status : Pehla enhu tan rakhla";    
    int index = ui_.pick_select->currentIndex();
    int count = ui_.pick_select->count();
    const QString str = ui_.pick_select->currentText();
    string str2 = str.toStdString(); 
    //ROS_INFO("No. of elements in pick combobox : %d",count);
    if(index == -1 || index == 0)
        ui_.error->setText(err1);
    else
    {
        if(pick_bool == 0)
        {    
            /*if(index == count - 1)
            {
                spatula_pick == 1;
            }*/
            if(index == 24)
            {
                stir_bool = 1;
            }
            pickit(index,str2);
            //pickit(str)
            pick_bool = 1; //---------------------bang bang uncomment it----------------------
            ROS_INFO("pick check 1");
            previousplace = index;
            previousname = str2;
            ROS_INFO("pick check 2");

        }
        else
            ui_.error->setText(err2);

    }
    err1.squeeze();
    err2.squeeze();

 /*   initializeLimits();
    ROS_INFO("Enter Check");
    moveit::planning_interface::MoveGroup group("five_dof_arm");
    geometry_msgs::Pose target1;
    geometry_msgs::PoseStamped temp;
    float yaw;
    bool b;
    std::vector<double> solution ;
    temp = group.getCurrentPose();
        ROS_INFO("Enter Check2 %f", temp.pose.position.x);
    float x = temp.pose.position.x + 0.2;
    float y = 0;
  
    target1.position.x = 0.11168;
    target1.position.y = 0.000004;
    target1.position.z = 0.558193;
    yaw = atan2(y,x);

    target1.orientation.z = 0.00003;
    target1.orientation.y = -0.706926; 
    target1.orientation.x = -0.00003; 
    target1.orientation.w = 0.707287; 
    //target1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0) ;
    ROS_INFO("X %f", target1.position.x);
    ROS_INFO("Y %f", target1.position.y);
    ROS_INFO("Z %f", target1.position.z);
    ROS_INFO("W %f", target1.orientation.w);
    ROS_INFO("Ox %f", target1.orientation.x);
    ROS_INFO("Oy %f", target1.orientation.y);
    ROS_INFO("Oz %f", target1.orientation.z);

    ROS_INFO("Enter Check3");     
      group.setPoseTarget(target1);
    solution = checkIK(target1,b) ;
    ROS_INFO("Enter Check4"); 
    ROS_INFO_STREAM("Joint1: " << solution[0]) ;
    ROS_INFO_STREAM("Joint2: " << solution[1]) ;
    ROS_INFO_STREAM("Joint3: " << solution[2]) ;
    ROS_INFO_STREAM("Joint4: " << solution[3]) ;
    ROS_INFO_STREAM("Joint5: " << solution[4]) ;
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
*/
}
void MyPlugin::place()
{
    QString err1 = "Status : Please select where to place";
    QString err2 = "Status : Please pick something first";
    int ind = ui_.place_select->currentIndex();
    //ROS_INFO("Command to pick up box%d",index);
    if(ind == -1 || ind == 0)
        ui_.error->setText(err1);
    else if(pick_bool == 1)
    { 
        if(ind == 1)
            {
                if(previousplace == 24)
                {
                    stir_bool = 0;
                }
                place_it(previousplace,previousname);
                pick_bool = 0;
            }
        else
            ;
    }
    else 
        ui_.error->setText(err2);
}
void MyPlugin::dispense() 
{
    MYSQL *connection = mysql_init(NULL);
    connection = mysql_real_connect(connection, "localhost", "root", "nymble", "nymble", 0, NULL, 0);
    MYSQL_RES *result;  
    MYSQL_ROW row;

    const char * c;
    std::string int2str;
    std::string s;
    std::string str;
    stringstream ss;
    std::ostringstream ss2;
    float pconst = 0.005;
    float d;
    QString err1 = "Status : Please pick something first to dispense";
    
    if(pick_bool == 1)
    {    
        double index = ui_.dispense_quant->value();
        int temp = (int)index;
        dispenseit(temp);
        //-------------------Index though not yet used-------------------
        ss.str(std::string());
        ss << previousplace;
        int2str = ss.str();
        //---------------------------------------------------------------
        s = "SELECT * FROM objects where name = '";
        str = s + previousname + "'";
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
            ROS_INFO("Value of qaunt %f", atof(row[11]));
            d = float(atof(row[11])) - float(pconst*temp);
            ROS_INFO("New Value of qaunt %f", d);            
            ss2 << d;
            std::string double2str(ss2.str());
            s = "UPDATE objects SET q = " + double2str + " where id = " + int2str;
            c = s.c_str();  
            ROS_INFO("%s",c);
            if(mysql_query(connection, c)) 
            {      
                ROS_INFO("Query Error: %s", mysql_error(connection));  
                exit(1);  
            }  
        }             
    }
    else
        ui_.error->setText(err1);        
}
void MyPlugin::stir()
{
    QString err1 = "Status : Please pick spatula first";
    int temp = ui_.stir_num->value();
    if(stir_bool == 1)
    {
        stir_it(temp);
    }
    else
    {
        ui_.error->setText(err1);        
    }
}
void MyPlugin::home()
{
    moveit::planning_interface::MoveGroup group("chef_arm");
    std::vector<double> solution;
    solution.push_back(0);
    solution.push_back(1.57);
    solution.push_back(-1.57);
    solution.push_back(0);
    solution.push_back(0);
    solution.push_back(0);

    jspinners(solution);
    jsliders(solution);

    group.setJointValueTarget(solution) ;
    group.move();
    solution.clear();

    psliders();
    pspinners();
}
void MyPlugin::jointChange() // change in joint angle via slider
{
    ROS_INFO("JointChange slider in action");
    moveit::planning_interface::MoveGroup group("chef_arm");
    std::vector<double> solution;
    solution.push_back(float(ui_.j1_slider->value())/float(100));
    solution.push_back(float(ui_.j2_slider->value())/float(100));
    solution.push_back(-float(ui_.j2_slider->value())/float(100));
    solution.push_back(float(ui_.j3_slider->value())/float(100));
    solution.push_back(-float(ui_.j3_slider->value())/float(100));
    solution.push_back(float(ui_.j4_slider->value())/float(100));

    jspinners(solution);

    group.setJointValueTarget(solution) ;
    group.move();
    solution.clear();

    psliders();
    pspinners();
}
void MyPlugin::jointChange2() // change in joint angle via spinners
{
    if(valuechange_backend == 0)
    {
        ROS_INFO("JointChange spinner in action");
        moveit::planning_interface::MoveGroup group("chef_arm");
        std::vector<double> solution;
        solution.push_back(ui_.j1_spin->value());
        solution.push_back(ui_.j2_spin->value());
        solution.push_back(-(ui_.j2_spin->value()));
        solution.push_back(ui_.j3_spin->value());
        solution.push_back(-(ui_.j3_spin->value()));
        solution.push_back(ui_.j4_spin->value());
        
        jsliders(solution);

        group.setJointValueTarget(solution) ;
        group.move() ;
        solution.clear();

        geometry_msgs::PoseStamped temp;
        temp = group.getCurrentPose();
        std::vector<double> temp2;
        temp2 = group.getCurrentRPY();
        float roll = temp2[0];
        float pitch = temp2[1];
        float yaw = temp2[2];

        psliders();
        pspinners();
    }
}
void MyPlugin::positionChange() // change in position via sliders
{
    initializeLimits();
    ROS_INFO("PositionChange slider in action");

    moveit::planning_interface::MoveGroup group("chef_arm");
    geometry_msgs::Pose target1;
    float yaw;
    bool b;
    std::vector<double> solution ;
    float x = float(ui_.x_slider->value())/float(100);
    float y = float(ui_.y_slider->value())/float(100);
    float z = float(ui_.z_slider->value())/float(100);
    float roll = float(ui_.r_slider->value())/float(100);
    float pitch = float(ui_.p_slider->value())/float(100);
    target1.position.x = x;
    target1.position.y = y;
    target1.position.z = z;
    yaw = atan2(y,x);

    ROS_INFO("Yaw %f", yaw);
    ROS_INFO("Pitch %f", pitch);

    target1.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw) ;
    ROS_INFO("X %f", target1.position.x);
    ROS_INFO("Y %f", target1.position.y);
    ROS_INFO("Z %f", target1.position.z);
    ROS_INFO("W %f", target1.orientation.w);
    ROS_INFO("Ox %f", target1.orientation.x);
    ROS_INFO("Oy %f", target1.orientation.y);
    ROS_INFO("Oz %f", target1.orientation.z);
    solution = checkIK(target1,b); 
    ROS_INFO_STREAM("Joint1: " << solution[0]) ;
    ROS_INFO_STREAM("Joint2: " << solution[1]) ;
    ROS_INFO_STREAM("Joint3: " << solution[3]) ;
    ROS_INFO_STREAM("Joint4: " << solution[5]) ;

    if (b)
    {    
        jsliders(solution);
        jspinners(solution);

        ROS_INFO_STREAM("Target is REACHABLE") ;
        group.setJointValueTarget(solution) ;
        group.move() ;

        pspinners();
    }
    else
    {
        ROS_INFO_STREAM("Target is OUT OF REACH") ;
    }
}
void MyPlugin::yawChange() // change in yaw via slider
{
    initializeLimits();
    ROS_INFO("PositionChange yaw slider in action");

    moveit::planning_interface::MoveGroup group("chef_arm");
    geometry_msgs::Pose target1;
    float yaw = float(ui_.yaw_slider->value())/float(100);
    bool b;
    std::vector<double> solution ;
    float x = float(ui_.x_slider->value())/float(100);
    float y = float(ui_.y_slider->value())/float(100);
    float dist = sqrt(x*x + y*y);
    
    float targetx = dist/(sqrt(1 + ((tan(yaw)*tan(yaw)))));
    float targety = targetx*tan(yaw);    
    float z = float(ui_.z_slider->value())/float(100);
    
    float roll = float(ui_.r_slider->value())/float(100);
    float pitch = float(ui_.p_slider->value())/float(100);
    
    target1.position.x = targetx;
    target1.position.y = targety;
    target1.position.z = z;

    target1.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw) ;
    ROS_INFO("X %f", target1.position.x);
    ROS_INFO("Y %f", target1.position.y);
    ROS_INFO("Z %f", target1.position.z);
    ROS_INFO("W %f", target1.orientation.w);
    ROS_INFO("Ox %f", target1.orientation.x);
    ROS_INFO("Oy %f", target1.orientation.y);
    ROS_INFO("Oz %f", target1.orientation.z);
    solution = checkIK(target1,b); 
    ROS_INFO_STREAM("Joint1: " << solution[0]) ;
    ROS_INFO_STREAM("Joint2: " << solution[1]) ;
    ROS_INFO_STREAM("Joint3: " << solution[3]) ;
    ROS_INFO_STREAM("Joint4: " << solution[5]) ;
    if (b)
    {    
        jsliders(solution);
        jspinners(solution);

        ROS_INFO_STREAM("Target is REACHABLE") ;
        group.setJointValueTarget(solution) ;
        group.move() ;

        pspinners();
    }
    else
    {
        ROS_INFO_STREAM("Target is OUT OF REACH") ;
    }
}

void MyPlugin::positionChange2() // change in positions via spinners
{
    if(valuechange_backend == 0)
    {
        initializeLimits();
        ROS_INFO("PositionChange spinner in action");

        moveit::planning_interface::MoveGroup group("chef_arm");
        geometry_msgs::Pose target1;
        float yaw;
        bool b;
        std::vector<double> solution ;
        float x = float(ui_.x_spin->value());
        float y = float(ui_.y_spin->value());
        float z = float(ui_.z_spin->value());
        float roll = float(ui_.r_spin->value());
        float pitch = float(ui_.p_spin->value());
        target1.position.x = x;
        target1.position.y = y;
        target1.position.z = z;
        yaw = atan2(y,x);

        ROS_INFO("Yaw %f", yaw);
        ROS_INFO("Pitch %f", pitch);

        target1.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw) ;
        ROS_INFO("X %f", target1.position.x);
        ROS_INFO("Y %f", target1.position.y);
        ROS_INFO("Z %f", target1.position.z);
        ROS_INFO("W %f", target1.orientation.w);
        ROS_INFO("Ox %f", target1.orientation.x);
        ROS_INFO("Oy %f", target1.orientation.y);
        ROS_INFO("Oz %f", target1.orientation.z);
        solution = checkIK(target1,b); 
        ROS_INFO_STREAM("Joint1: " << solution[0]) ;
        ROS_INFO_STREAM("Joint2: " << solution[1]) ;
        ROS_INFO_STREAM("Joint3: " << solution[3]) ;
        ROS_INFO_STREAM("Joint4: " << solution[5]) ;

        if (b)
        {    
            ui_.x_slider->setValue(int(x*100));
            ui_.y_slider->setValue(int(y*100));
            ui_.z_slider->setValue(int(z*100));
            ui_.r_slider->setValue(int(roll*100));
            ui_.p_slider->setValue(int(pitch*100));
            ui_.yaw_slider->setValue(int(yaw*100));

            jsliders(solution);
            jspinners(solution);

            ROS_INFO_STREAM("Target is REACHABLE") ;
            group.setJointValueTarget(solution) ;
            group.move() ;
        }
        else
        {
            ROS_INFO_STREAM("Target is OUT OF REACH") ;
        }
    }
}
void MyPlugin::yawChange2() // change in yaw via spinner
{
    if(valuechange_backend == 0)
    {
        initializeLimits();
        ROS_INFO("PositionChange spinner yaw in action");

        moveit::planning_interface::MoveGroup group("chef_arm");
        geometry_msgs::Pose target1;
        float yaw = float(ui_.yaw_spin->value());
        bool b;
        std::vector<double> solution ;
        float x = float(ui_.x_spin->value());
        float y = float(ui_.y_spin->value());
        float dist = sqrt(x*x + y*y);
        
        float targetx = dist/(sqrt(1 + ((tan(yaw)*tan(yaw)))));
        float targety = targetx*tan(yaw);    
        float z = float(ui_.z_spin->value());
        
        float roll = float(ui_.r_spin->value());
        float pitch = float(ui_.p_spin->value());
        
        target1.position.x = targetx;
        target1.position.y = targety;
        target1.position.z = z;

        target1.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw) ;
        ROS_INFO("X %f", target1.position.x);
        ROS_INFO("Y %f", target1.position.y);
        ROS_INFO("Z %f", target1.position.z);
        ROS_INFO("W %f", target1.orientation.w);
        ROS_INFO("Ox %f", target1.orientation.x);
        ROS_INFO("Oy %f", target1.orientation.y);
        ROS_INFO("Oz %f", target1.orientation.z);
        solution = checkIK(target1,b); 
        ROS_INFO_STREAM("Joint1: " << solution[0]) ;
        ROS_INFO_STREAM("Joint2: " << solution[1]) ;
        ROS_INFO_STREAM("Joint3: " << solution[3]) ;
        ROS_INFO_STREAM("Joint4: " << solution[5]) ;
        
        if (b)
        {    
            ui_.x_slider->setValue(int(targetx*100));
            ui_.y_slider->setValue(int(targety*100));
            ui_.z_slider->setValue(int(z*100));
            ui_.r_slider->setValue(int(roll*100));
            ui_.p_slider->setValue(int(pitch*100));
            ui_.yaw_slider->setValue(int(yaw*100));

            jsliders(solution);
            jspinners(solution);

            ROS_INFO_STREAM("Target is REACHABLE") ;
            group.setJointValueTarget(solution) ;
            group.move() ;
        }
        else
        {
            ROS_INFO_STREAM("Target is OUT OF REACH") ;
        }
    }
}
void MyPlugin::vacuum()
{
    if(vacuum_bool == 0)
    {
        vacuum_bool = 1;
        ui_.vacuum_status -> setText("ON");
        ui_.redbut -> setStyleSheet("");
        ui_.greenbut -> setStyleSheet("background-color: rgb(0, 255, 0);"); 
    }
    else
    {
        ui_.redbut -> setStyleSheet("background-color: rgb(255, 0, 0);");
        ui_.greenbut -> setStyleSheet(""); 
        vacuum_bool = 0;
        ui_.vacuum_status -> setText("OFF");
    }
}
void MyPlugin::jsliders(std::vector<double> solution)
{
    ui_.j1_slider->setValue(int(solution[0]*100));
    ui_.j2_slider->setValue(int(solution[1]*100));
    ui_.j3_slider->setValue(int(solution[3]*100));
    ui_.j4_slider->setValue(int(solution[5]*100));
}
void MyPlugin::jspinners(std::vector<double> solution)
{
    valuechange_backend = 1;
    ui_.j1_spin->setValue(solution[0]);
    ui_.j2_spin->setValue(solution[1]);
    ui_.j3_spin->setValue(solution[3]);
    ui_.j4_spin->setValue(solution[5]);
    valuechange_backend = 0;    
}
void MyPlugin::psliders()
{
    moveit::planning_interface::MoveGroup group("chef_arm");
    geometry_msgs::PoseStamped temp;
    temp = group.getCurrentPose();
    std::vector<double> temp2;
    temp2 = group.getCurrentRPY();
    float roll = temp2[0];
    float pitch = temp2[1];
    float yaw = temp2[2];

    ui_.x_slider->setValue(int(temp.pose.position.x*100));
    ui_.y_slider->setValue(int(temp.pose.position.y*100));
    ui_.z_slider->setValue(int(temp.pose.position.z*100));
    ui_.r_slider->setValue(int(roll*100));
    ui_.p_slider->setValue(int(pitch*100));
    ui_.yaw_slider->setValue(int(yaw*100));
}
void MyPlugin::pspinners()
{
    valuechange_backend = 1;
    moveit::planning_interface::MoveGroup group("chef_arm");    
    geometry_msgs::PoseStamped temp;
    temp = group.getCurrentPose();
    std::vector<double> temp2;
    temp2 = group.getCurrentRPY();
    float roll = temp2[0];
    float pitch = temp2[1];
    float yaw = temp2[2];

    ui_.x_spin->setValue(temp.pose.position.x);
    ui_.y_spin->setValue(temp.pose.position.y);
    ui_.z_spin->setValue(temp.pose.position.z);
    ui_.r_spin->setValue(roll);
    ui_.p_spin->setValue(pitch);
    ui_.yaw_spin->setValue(yaw);
    valuechange_backend = 0;
}

void MyPlugin::setupROSComponents()
{
    //-----------------------------------------Temperature Subscribing from Temperature topic---------------------------------
    sub = getNodeHandle().subscribe<std_msgs::String>("temp", 1, &MyPlugin::temperature, this);

    ROS_INFO("Ya yha to phoncha hai");
  /*    pub_cmd_vel_ = getNodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    srv_switchCamera_ = getNodeHandle()
        .serviceClient<std_srvs::Empty>("/ardrone/togglecam");
  */
}


} // namespace




PLUGINLIB_EXPORT_CLASS(rqt_mypkg::MyPlugin, rqt_gui_cpp::Plugin)
