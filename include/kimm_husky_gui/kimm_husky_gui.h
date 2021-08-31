#ifndef kimm_husky_gui__HuskyGUI_H
#define kimm_husky_gui__HuskyGUI_H

#include <rqt_gui_cpp/plugin.h>
#include "kimm_husky_gui/ui_kimm_husky_gui.h"

#include <ros/macros.h>

#include <QList>
#include <QString>
#include <QSize>
#include <QWidget>
#include <QObject>
#include <QStateMachine>
#include <QState>
#include <QEventTransition>
#include <QMetaType>
#include <QGraphicsRectItem>
#include <QGraphicsSceneWheelEvent>
#include <QStringListModel>
#include <QSignalMapper>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include "mujoco_ros_msgs/JointSet.h"
#include <geometry_msgs/Transform.h>


#include <Eigen/Dense>
#define DEGREE 180.0/M_PI

namespace kimm_husky_gui
{
    class HuskyGui : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT
    public:
        HuskyGui();
        virtual void initPlugin(qt_gui_cpp::PluginContext& context);
        virtual void shutdownPlugin(){};
        virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const {};
        virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings) {};

    private:
        double time_;
        Ui::HuskyGuiWidget ui_;
        QWidget *widget_;

        ros::NodeHandle nh_;
        std_msgs::Bool bool_msg_;
        std_msgs::Int16 int16_msg_;
        Eigen::Quaternion<double> _quat;
        Eigen::Vector3d _angles;

    protected slots:
        protected slots:
        virtual void EnableSimulCallback(){
            bool_msg_.data = true;
            run_pub_.publish(bool_msg_);
        };
        virtual void DisableSimulCallback(){
            bool_msg_.data = false;
            run_pub_.publish(bool_msg_);
        };
        virtual void InitCallback(){
            int16_msg_.data = 1;
            custom_ctrl_pub_.publish(int16_msg_);
        };
        virtual void GravityCallback(){
            int16_msg_.data = 0;
            custom_ctrl_pub_.publish(int16_msg_);
        };
        virtual void CustomCtrl1Callback(){
            int16_msg_.data = 1;
            custom_ctrl_pub_.publish(int16_msg_);
        };
        virtual void CustomCtrl2Callback(){
            int16_msg_.data = 2;
            custom_ctrl_pub_.publish(int16_msg_);
        };
        virtual void CustomCtrl3Callback(){
            int16_msg_.data = 3;
            custom_ctrl_pub_.publish(int16_msg_);
        };
        virtual void CustomCtrl4Callback(){
            int16_msg_.data = 4;
            custom_ctrl_pub_.publish(int16_msg_);
        };
        virtual void CustomCtrl5Callback(){
            int16_msg_.data = 5;
            custom_ctrl_pub_.publish(int16_msg_);
        };
        virtual void CustomCtrl6Callback(){
            int16_msg_.data = 6;
            custom_ctrl_pub_.publish(int16_msg_);
        };
        virtual void CustomCtrl7Callback(){
            int16_msg_.data = 7;
            custom_ctrl_pub_.publish(int16_msg_);
        };
        virtual void CustomCtrl8Callback(){
            int16_msg_.data = 8;
            custom_ctrl_pub_.publish(int16_msg_);
        };
        virtual void CustomCtrl9Callback(){
            int16_msg_.data = 9;
            custom_ctrl_pub_.publish(int16_msg_);
        };
        virtual void CustomCtrl10Callback(){
            int16_msg_.data = 10;
            custom_ctrl_pub_.publish(int16_msg_);
        };


        virtual void Timercb(const std_msgs::Float32ConstPtr &msg){
            time_ = msg->data;
            ui_.currenttime->setText(QString::number(msg->data, 'f', 3));
        };
        virtual void Jointcb(const sensor_msgs::JointStateConstPtr &msg){
            ui_.robot_p1->setText(QString::number(msg->position[7] * DEGREE , 'f', 3));
            ui_.robot_p2->setText(QString::number(msg->position[8] * DEGREE , 'f', 3));
            ui_.robot_p3->setText(QString::number(msg->position[11] * DEGREE , 'f', 3));
            ui_.robot_p4->setText(QString::number(msg->position[12] * DEGREE , 'f', 3));
            ui_.robot_p5->setText(QString::number(msg->position[13] * DEGREE , 'f', 3));
            ui_.robot_p6->setText(QString::number(msg->position[14] * DEGREE , 'f', 3));
            ui_.robot_p7->setText(QString::number(msg->position[15] * DEGREE , 'f', 3));
            ui_.robot_p8->setText(QString::number(msg->position[16] * DEGREE , 'f', 3));
            ui_.robot_p9->setText(QString::number(msg->position[17] * DEGREE , 'f', 3));

            ui_.robot_v1->setText(QString::number(msg->velocity[6] * DEGREE , 'f', 3));
            ui_.robot_v2->setText(QString::number(msg->velocity[7] * DEGREE , 'f', 3));
            ui_.robot_v3->setText(QString::number(msg->velocity[10] * DEGREE , 'f', 3));
            ui_.robot_v4->setText(QString::number(msg->velocity[11] * DEGREE , 'f', 3));
            ui_.robot_v5->setText(QString::number(msg->velocity[12] * DEGREE , 'f', 3));
            ui_.robot_v6->setText(QString::number(msg->velocity[13] * DEGREE , 'f', 3));
            ui_.robot_v7->setText(QString::number(msg->velocity[14] * DEGREE , 'f', 3));
            ui_.robot_v8->setText(QString::number(msg->velocity[15] * DEGREE , 'f', 3));
            ui_.robot_v9->setText(QString::number(msg->velocity[16] * DEGREE , 'f', 3));
        }; 
        virtual void Torquecb(const mujoco_ros_msgs::JointSetConstPtr &msg){
            ui_.robot_t1->setText(QString::number(msg->torque[0] , 'f', 3));
            ui_.robot_t2->setText(QString::number(msg->torque[1] , 'f', 3));
            ui_.robot_t3->setText(QString::number(msg->torque[4] , 'f', 3));
            ui_.robot_t4->setText(QString::number(msg->torque[5] , 'f', 3));
            ui_.robot_t5->setText(QString::number(msg->torque[6] , 'f', 3));
            ui_.robot_t6->setText(QString::number(msg->torque[7] , 'f', 3));
            ui_.robot_t7->setText(QString::number(msg->torque[8] , 'f', 3));
            ui_.robot_t8->setText(QString::number(msg->torque[9] , 'f', 3));      
            ui_.robot_t9->setText(QString::number(msg->torque[10] , 'f', 3));                     
        }; 
        virtual void Basecb(const sensor_msgs::JointStateConstPtr& msg){
            ui_.base_p1->setText(QString::number(msg->position[0] , 'f', 3));
            ui_.base_p2->setText(QString::number(msg->position[1] , 'f', 3));
            ui_.base_p3->setText(QString::number(msg->position[2] * DEGREE, 'f', 3));
        };
        virtual void EEcb(const geometry_msgs::TransformConstPtr& msg){
            ui_.ee_p1->setText(QString::number(msg->translation.x , 'f', 3));
            ui_.ee_p2->setText(QString::number(msg->translation.y , 'f', 3));
            ui_.ee_p3->setText(QString::number(msg->translation.z , 'f', 3));

            _quat.x() = msg->rotation.x;
            _quat.y() = msg->rotation.y;
            _quat.z() = msg->rotation.z;
            _quat.w() = msg->rotation.w;

            _angles = _quat.toRotationMatrix().eulerAngles(0,1,2);
            ui_.ee_p4->setText(QString::number(_angles(0) * DEGREE , 'f', 3));
            ui_.ee_p5->setText(QString::number(_angles(1) * DEGREE , 'f', 3));
            ui_.ee_p6->setText(QString::number(_angles(2) * DEGREE , 'f', 3));
        };

    signals:
        void timerCallback(const std_msgs::Float32ConstPtr &msg);
        void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
        void torqueStateCallback(const mujoco_ros_msgs::JointSetConstPtr & msg);
        void baseStateCallback(const sensor_msgs::JointStateConstPtr& msg);
        void eeStateCallback(const geometry_msgs::TransformConstPtr& msg);

    protected slots:
        virtual void customctrlbtn(){
            ui_.stackedWidget->setCurrentIndex(0);
        };
        virtual void jointctrlbtn(){
            ui_.stackedWidget->setCurrentIndex(1);
        };
        virtual void se3ctrlbtn(){
            ui_.stackedWidget->setCurrentIndex(2);
        };
        virtual void basectrlbtn(){
            ui_.stackedWidget->setCurrentIndex(3);
        };

    public:
        ros::Publisher run_pub_, custom_ctrl_pub_;
        ros::Subscriber simtime_sub_, jointstate_sub_, torquestate_sub_, base_state_sub_, ee_state_sub_;
        


    };

}
 // namespace kimm_husky_gui
Q_DECLARE_METATYPE(std_msgs::StringConstPtr);
Q_DECLARE_METATYPE(std_msgs::Float32ConstPtr);
Q_DECLARE_METATYPE(std_msgs::Float64ConstPtr);
Q_DECLARE_METATYPE(sensor_msgs::JointStateConstPtr);
Q_DECLARE_METATYPE(mujoco_ros_msgs::JointSetConstPtr);
Q_DECLARE_METATYPE(geometry_msgs::TransformConstPtr);


#endif
