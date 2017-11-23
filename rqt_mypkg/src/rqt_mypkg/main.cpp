#include "rqt_mypkg/my_plugin.h"

#include <QMainWindow>
#include <QApplication>
#include <QDockWidget>
#include <QWidget>

#include <ros/ros.h>

using namespace rqt_mypkg;

int main( int argc, char **argv ) {

    // Create an application
    QApplication a( argc, argv );

//    ros::init(argc, argv, "rqt_mypkg");

    // Create main window
    QMainWindow w;

    // new rqt plugin
    MyPlugin* myPlugin;
    myPlugin = new MyPlugin();

    // 1. attempt by casting myPlugin to a QDockWidget
    QDockWidget *dockWidget = qobject_cast<QDockWidget*>(myPlugin);
    dockWidget->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    w.addDockWidget(Qt::RightDockWidgetArea, dockWidget);

    // 2. attempt by csting myPlugin to a QWidget and add it to a QDockWidget
//    QWidget *widget = qobject_cast<QWidget*>(myPlugin);
//    QDockWidget *dock = new QDockWidget();
//    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
//    dock->setWidget(widget);
//    w.addDockWidget(Qt::RightDockWidgetArea, dock);

    // show main window
    w.show();
    return a.exec();
}
