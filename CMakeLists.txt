cmake_minimum_required(VERSION 3.5.0)
project(kimm_husky_gui)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  sensor_msgs
  std_msgs
  geometry_msgs
  mujoco_ros_msgs
)



find_package(Qt5 REQUIRED Core Widgets X11Extras Gui)
find_package(Eigen3)

set(CMAKE_AUTOMOC ON)
catkin_python_setup()

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp sensor_msgs std_msgs geometry_msgs mujoco_ros_msgs kimm_joint_planner_ros_interface kimm_path_planner_ros_interface kimm_se3_planner_ros_interface
  #DEPENDS system_lib
)



set(husky_gui_SRCS
  src/kimm_husky_gui.cpp
)

set(husky_gui_HDRS
  include/kimm_husky_gui/kimm_husky_gui.h
)

set(husky_gui_UIS
  src/kimm_husky_gui.ui
)

set(qxtglobalshortcut
  core/qxtglobal.cpp
  gui/qxtwindowsystem.cpp
  gui/qxtglobalshortcut.cpp
  gui/qxtwindowsystem_x11.cpp
  gui/qxtglobalshortcut_x11.cpp
)


qt5_wrap_cpp(husky_gui_MOCS ${husky_gui_HDRS})
qt5_wrap_ui(husky_gui_UIS_H ${husky_gui_UIS})


###########
## Build ##
###########

include_directories(
  include
  core
  gui
  ${Qt5Gui_PRIVATE_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${husky_gui_SRCS}
  ${husky_gui_MOCS}
  ${husky_gui_UIS_H}
  ${qxtglobalshortcut}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Core Qt5::Gui Qt5::X11Extras Qt5::GuiPrivate)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
