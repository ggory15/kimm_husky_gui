
#include "kimm_husky_gui/kimm_husky_gui.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <vector>
#include <QDebug>
#include "qxtglobalshortcut.h"



namespace kimm_husky_gui
{
   HuskyGui::HuskyGui()
   : rqt_gui_cpp::Plugin(), widget_(0)
    {
        qRegisterMetaType<std_msgs::Float32ConstPtr>();
        qRegisterMetaType<std_msgs::Float64ConstPtr>();
        qRegisterMetaType<sensor_msgs::JointStateConstPtr>();
        qRegisterMetaType<mujoco_ros_msgs::JointSetConstPtr>();
        qRegisterMetaType<geometry_msgs::TransformConstPtr>();

        setObjectName("HuskyGui");

        run_pub_ = nh_.advertise<std_msgs::Bool>("/mujoco_ros_interface/sim_run", 100);
        custom_ctrl_pub_ = nh_.advertise<std_msgs::Int16>("/mujoco_ros_interface/ctrl_type", 100);
        base_traj_resp_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/ns0/kimm_mobile_plan_markers/mobile/response", 100);
        base_traj_req_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/ns0/kimm_mobile_plan_markers/mobile/request", 100);
        ee_traj_resp_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/ns0/kimm_se3_plan_markers/robot/response", 100);
        // obs_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/ns0/kimm_mobile_plan_markers/mobile/request", 100);

        simtime_sub_ = nh_.subscribe("/mujoco_ros_interface/sim_time", 100, &HuskyGui::timerCallback, this);
        jointstate_sub_ = nh_.subscribe("/mujoco_ros_interface/joint_states", 100, &HuskyGui::jointStateCallback, this);
        torquestate_sub_ = nh_.subscribe("/mujoco_ros_interface/joint_set", 100, &HuskyGui::torqueStateCallback, this);
        base_state_sub_ = nh_.subscribe("ns0/mujoco_ros_interface/base_state", 100, &HuskyGui::baseStateCallback, this);
        ee_state_sub_ = nh_.subscribe("ns0/mujoco_ros_interface/ee_state", 100, &HuskyGui::eeStateCallback, this);

        joint_plan_client_ = nh_.serviceClient<kimm_joint_planner_ros_interface::plan_joint_path>("/ns0/kimm_joint_planner_ros_interface_server/plan_joint_path");
        mobile_plan_client_ = nh_.serviceClient<kimm_path_planner_ros_interface::plan_mobile_path>("/ns0/kimm_path_planner_ros_interface_server/plan_mobile_path");
        se3_plan_client_ = nh_.serviceClient<kimm_se3_planner_ros_interface::plan_se3_path>("/ns0/kimm_se3_planner_ros_interface_server/plan_se3_path");
       
        q_.setZero(9);
        x_.setZero(6);
    }
    void HuskyGui::initPlugin(qt_gui_cpp::PluginContext& context)
    {
        widget_ = new QWidget();
        ui_.setupUi(widget_);

        if (context.serialNumber() > 1)
        {
            widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
        }
        context.addWidget(widget_);

        ui_.simulon_button->setShortcut(QKeySequence(Qt::Key_R));
        ui_.simuloff_button->setShortcut(QKeySequence(Qt::Key_P));
        ui_.grasp_button->setShortcut(QKeySequence(Qt::Key_G));

        // Basic Interface from Mujoco
        connect(this, &HuskyGui::timerCallback, this, &HuskyGui::Timercb);
        connect(this, &HuskyGui::jointStateCallback, this, &HuskyGui::Jointcb);
        connect(this, &HuskyGui::torqueStateCallback, this, &HuskyGui::Torquecb);
        connect(this, &HuskyGui::baseStateCallback, this, &HuskyGui::Basecb);
        connect(this, &HuskyGui::eeStateCallback, this, &HuskyGui::EEcb);

        // Basic Interface to Mujoco
        connect(ui_.simulon_button, SIGNAL(pressed()), this, SLOT(EnableSimulCallback()));
        connect(ui_.simuloff_button, SIGNAL(pressed()), this, SLOT(DisableSimulCallback()));
        connect(ui_.init_button, SIGNAL(pressed()), this, SLOT(InitCallback()));
        connect(ui_.grasp_button, SIGNAL(pressed()), this, SLOT(GraspCallback()));
       
        // Custom Controller Interface to Mujoco
        connect(ui_.custom_ctrl_btn, SIGNAL(pressed()), this, SLOT(customctrlbtn()));
        ui_.custom_ctrl_btn->setShortcut(QKeySequence(Qt::Key_1));
        connect(ui_.Customctrlbtn_1, SIGNAL(pressed()), this, SLOT(CustomCtrl1Callback()));
        connect(ui_.Customctrlbtn_2, SIGNAL(pressed()), this, SLOT(CustomCtrl2Callback()));
        connect(ui_.Customctrlbtn_3, SIGNAL(pressed()), this, SLOT(CustomCtrl3Callback()));
        connect(ui_.Customctrlbtn_4, SIGNAL(pressed()), this, SLOT(CustomCtrl4Callback()));
        connect(ui_.Customctrlbtn_5, SIGNAL(pressed()), this, SLOT(CustomCtrl5Callback()));
        connect(ui_.Customctrlbtn_6, SIGNAL(pressed()), this, SLOT(CustomCtrl6Callback()));
        connect(ui_.Customctrlbtn_7, SIGNAL(pressed()), this, SLOT(CustomCtrl7Callback()));
        connect(ui_.Customctrlbtn_8, SIGNAL(pressed()), this, SLOT(CustomCtrl8Callback()));
        connect(ui_.Customctrlbtn_9, SIGNAL(pressed()), this, SLOT(CustomCtrl9Callback()));
        connect(ui_.Customctrlbtn_10, SIGNAL(pressed()), this, SLOT(CustomCtrl10Callback()));

        // Joint Controller Interface to Mujoco
        connect(ui_.joint_ctrl_btn, SIGNAL(pressed()), this, SLOT(jointctrlbtn()));
        ui_.joint_ctrl_btn->setShortcut(QKeySequence(Qt::Key_2));
        connect(ui_.joint_trajectory_send, SIGNAL(pressed()), this, SLOT(jointctrlcb()));
        connect(ui_.joint_trajectory_send_2, SIGNAL(pressed()), this, SLOT(jointctrlcb2()));
        connect(ui_.c_joint, SIGNAL(pressed()), this, SLOT(jointctrlcb3()));
        connect(ui_.waypoint_add_btn, SIGNAL(pressed()), this, SLOT(jointctrlcb4()));
        connect(ui_.waypoint_clear_btn, SIGNAL(pressed()), this, SLOT(jointctrlcb5()));

        // SE3 Controller Interface to Mujoco
        connect(ui_.se3_ctrl_btn, SIGNAL(pressed()), this, SLOT(se3ctrlbtn()));
        ui_.se3_ctrl_btn->setShortcut(QKeySequence(Qt::Key_3));
        connect(ui_.se3_trajectory_send_1, SIGNAL(pressed()), this, SLOT(se3ctrlcb1()));
        connect(ui_.se3_trajectory_send_2, SIGNAL(pressed()), this, SLOT(se3ctrlcb2()));
        connect(ui_.c_joint_2, SIGNAL(pressed()), this, SLOT(se3ctrlcb3()));
        connect(ui_.waypoint_add_btn_2, SIGNAL(pressed()), this, SLOT(se3ctrlcb4()));
        connect(ui_.waypoint_clear_btn_2, SIGNAL(pressed()), this, SLOT(se3ctrlcb5()));

        // Base Controller Interface to Mujoco
        connect(ui_.base_ctrl_btn, SIGNAL(pressed()), this, SLOT(basectrlbtn()));
        ui_.base_ctrl_btn->setShortcut(QKeySequence(Qt::Key_4));
        connect(ui_.base_p_set_btn, SIGNAL(pressed()), this, SLOT(basectrlcb1())); // set configuration
        connect(ui_.base_p_run, SIGNAL(pressed()), this, SLOT(basectrlcb2())); // run planner
        connect(ui_.forward_btn, SIGNAL(pressed()), this, SLOT(basectrlcb3())); // run forward
        connect(ui_.backward_btn, SIGNAL(pressed()), this, SLOT(basectrlcb4())); // run backward
        connect(ui_.obs_add_btn, SIGNAL(pressed()), this, SLOT(basectrlcb5())); // obs add
        connect(ui_.obs_del_btn, SIGNAL(pressed()), this, SLOT(basectrlcb6())); // obs del
    }

} // namespace kimm_husky_gui

PLUGINLIB_EXPORT_CLASS(kimm_husky_gui::HuskyGui, rqt_gui_cpp::Plugin)
