cmake_minimum_required(VERSION 2.8.3)

project(rqt_mypkg)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake-modules) ####CHANGE
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)  ####CHANGE
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)  ####CHANGE

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
    rqt_gui 
    rqt_gui_cpp
             moveit_core
             moveit_ros_planning
             moveit_ros_planning_interface
             pluginlib
             cmake_modules
             geometric_shapes
             joint_trajectory_controller
             joint_trajectory_action
             moveit_simple_controller_manager
             industrial_robot_simulator
             actionlib
             actionlib_msgs
             control_msgs
             trajectory_msgs
             moveit_msgs
             shape_msgs
	           proto1_moveit
)

find_package(MySqlClient REQUIRED) ###CHANGE

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})

include_directories(${MYSQL_INCLUDE_DIRS}) ###CHANGE


# Flags
SET(CMAKE_CXX_FLAGS "-std=c++11")

include(${QT_USE_FILE})

set(rqt_mypkg_SRCS
  src/rqt_mypkg/my_plugin.cpp
)

set(rqt_mypkg_HDRS
  include/rqt_mypkg/my_plugin.h
)

set(rqt_mypkg_UIS
  src/rqt_mypkg/my_plugin.ui
)

set(rqt_mypkg_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    pick_proto
    place_proto
    dispense_proto
    stir_proto
  CATKIN_DEPENDS 
    rqt_gui 
    rqt_gui_cpp
    pick_proto 
    place_proto
    dispense_proto
    stir_proto
)
catkin_python_setup()

add_definitions(${MYSQL_DEFINITIONS}) ##CHANGE

qt4_wrap_cpp(rqt_mypkg_MOCS ${rqt_mypkg_HDRS})
qt4_wrap_ui(rqt_mypkg_UIS_H ${rqt_mypkg_UIS})

include_directories(${rqt_mypkg_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS} include)
add_library(${PROJECT_NAME} ${rqt_mypkg_SRCS} ${rqt_mypkg_HDRS} ${rqt_mypkg_MOCS} ${rqt_mypkg_UIS_H})
target_link_libraries(${PROJECT_NAME} 
    ${catkin_LIBRARIES} 
    ${QT_QTCORE_LIBRARY} 
    ${QT_QTGUI_LIBRARY} 
    ${MYSQL_LIBRARIES}
)
#add_dependencies(${PROJECT_NAME} 
#)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_mypkg
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/rqt_mypkg
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


add_executable(main
    src/rqt_mypkg/main.cpp
    ${rqt_mypkg_SRCS}
    ${rqt_mypkg_HDRS}
    ${rqt_mypkg_MOCS}
    ${rqt_mypkg_UIS_H}
)
target_link_libraries(main
    ${catkin_LIBRARIES}
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${MYSQL_LIBRARIES}
)
