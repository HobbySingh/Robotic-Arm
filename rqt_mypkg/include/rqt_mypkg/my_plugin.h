#pragma once

#include <rqt_gui_cpp/plugin.h>

#include <ui_my_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include <ros/master.h>
#include <ros/macros.h>

#include <QList>
#include <QString>
#include <QSize>
#include <QSlider>
#include <QWidget>
#include <QDoubleSpinBox>
#include <QObject>
#include <QEvent>
#include <QTableView>
#include <QResizeEvent>

#include <QApplication>
#include <QLabel>
#include <QPainter>

#include <vector>
#include <std_msgs/String.h>

namespace rqt_mypkg {

class MyPlugin
  : public rqt_gui_cpp::Plugin {

    Q_OBJECT

private:
    const char* TAG = "mission_monitor";
public:
    /*void setupROSComponents()
    {
        //-----------------------------------------Temperature Subscribing from Temperature topic---------------------------------
        sub = n.subscribe("temp", 1, &MyPlugin::temperature, this);
        pub = n.advertise<std_msgs::Header>("temp_proto",1000);
        ROS_INFO("Ya yha to phoncha hai");
    }*/
    int pick_bool = 0;
    int previousplace ;
    int stir_bool = 0;
    int valuechange_backend = 0;
    int spatula_pick = 0;
    int vacuum_bool = 0;
    std::string previousname;
    ros::Subscriber sub;
    ros::NodeHandle n; 
    ros::Publisher pub;
    void temperature(const std_msgs::String::ConstPtr&);

    MyPlugin();
    
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, 
                              qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, 
                                 const qt_gui_cpp::Settings& instance_settings);
public slots:
    void pick();
    void place();    
    void dispense();
    void stir();
    void home();
    void jointChange();
    void jointChange2();    
    void positionChange();
    void positionChange2();
    void yawChange();
    void yawChange2();
    void vacuum();
    void jsliders(std::vector<double>);
    void jspinners(std::vector<double>);
    void psliders();
    void pspinners();

protected slots:

protected:
    Ui::MyPluginWidget ui_;
    QWidget* widget_;
    void connectROSComponents();
    void setupROSComponents();
};
/*int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  setupROSComponents SAPObject;

  ros::spin();

  return 0;
}*/
}
