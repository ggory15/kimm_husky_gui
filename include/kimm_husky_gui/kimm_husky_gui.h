#ifndef kimm_husky_gui__HuskyGUI_H
#define kimm_husky_gui__HuskyGUI_H

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>


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
#include <geometry_msgs/Pose2D.h>

#include "kimm_joint_planner_ros_interface/plan_joint_path.h"
#include "kimm_path_planner_ros_interface/plan_mobile_path.h"
#include "kimm_path_planner_ros_interface/MobileTrajectory.h"
#include "kimm_path_planner_ros_interface/Obstacle2D.h"
#include "kimm_se3_planner_ros_interface/plan_se3_path.h"
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#define DEGREE 180.0/M_PI
#define RAD M_PI/180.0

using namespace std;
using namespace pinocchio;

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
        Model model_;    

        ros::Publisher joint_state_pub_;
        sensor_msgs::JointState joint_state_msg_;

    protected slots:
        protected slots:
        virtual void EnableSimulCallback(){
            bool_msg_.data = true;
            run_pub_.publish(bool_msg_);
        };
        virtual void DisableSimulCallback(){
            int16_msg_.data = 0;
            custom_ctrl_pub_.publish(int16_msg_);
        };
        virtual void InitCallback(){
            int16_msg_.data = 1;
            custom_ctrl_pub_.publish(int16_msg_);
        };
        virtual void QuitCallback(){
            bool_msg_.data = true;
            quit_pub_.publish(bool_msg_);
        }
        virtual void GraspCallback(){
            int16_msg_.data = 899;
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
            if (issimulation_){
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

                q_(0) = 0;
                q_(1) = 0;
                for (int i=0; i<7; i++)
                    q_(i+2) = msg->position[11+i];
            }
            else{
                ui_.robot_p1->setText(QString::number(msg->position[0] * DEGREE , 'f', 3));
                ui_.robot_p2->setText(QString::number(msg->position[1] * DEGREE , 'f', 3));
                ui_.robot_p3->setText(QString::number(msg->position[2] * DEGREE , 'f', 3));
                ui_.robot_p4->setText(QString::number(msg->position[3] * DEGREE , 'f', 3));
                ui_.robot_p5->setText(QString::number(msg->position[4] * DEGREE , 'f', 3));
                ui_.robot_p6->setText(QString::number(msg->position[5] * DEGREE , 'f', 3));
                ui_.robot_p7->setText(QString::number(msg->position[6] * DEGREE , 'f', 3));
                ui_.robot_p8->setText(QString::number(msg->position[7] * DEGREE , 'f', 3));
                ui_.robot_p9->setText(QString::number(msg->position[8] * DEGREE , 'f', 3));

                ui_.robot_v1->setText(QString::number(msg->velocity[0] * DEGREE , 'f', 3));
                ui_.robot_v2->setText(QString::number(msg->velocity[1] * DEGREE , 'f', 3));
                ui_.robot_v3->setText(QString::number(msg->velocity[2] * DEGREE , 'f', 3));
                ui_.robot_v4->setText(QString::number(msg->velocity[3] * DEGREE , 'f', 3));
                ui_.robot_v5->setText(QString::number(msg->velocity[4] * DEGREE , 'f', 3));
                ui_.robot_v6->setText(QString::number(msg->velocity[5] * DEGREE , 'f', 3));
                ui_.robot_v7->setText(QString::number(msg->velocity[6] * DEGREE , 'f', 3));
                ui_.robot_v8->setText(QString::number(msg->velocity[7] * DEGREE , 'f', 3));
                ui_.robot_v9->setText(QString::number(msg->velocity[8] * DEGREE , 'f', 3));

                q_(0) = 0;
                q_(1) = 0;
                for (int i=0; i<7; i++)
                    q_(i+2) = msg->position[11+i];
            }

            if (issimulation_){
                for (int i=0; i<4; i++)
                    joint_state_msg_.position[i] = msg->position[i+7];

                for (int i=0; i<7; i++)
                    joint_state_msg_.position[i+4] = msg->position[i+11];

                for (int i=0; i<2; i++)
                    joint_state_msg_.position[i+11] = msg->position[i+18];
            }
            else{
                joint_state_msg_.position[0] = msg->position[0];
                joint_state_msg_.position[2] = msg->position[0];
                joint_state_msg_.position[1] = msg->position[1];
                joint_state_msg_.position[3] = msg->position[1];

                for (int i=0; i<7; i++)
                    joint_state_msg_.position[i+4] = msg->position[i+2];

                for (int i=0; i<2; i++)
                    joint_state_msg_.position[i+11] = msg->position[i+9];
            }
                         
            joint_state_msg_.header.stamp = ros::Time::now();            
            joint_state_pub_.publish(joint_state_msg_);

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

            for (int i=0; i<3; i++)
                base_(i) = msg->position[i];

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
            
            x_(0) = msg->translation.x;
            x_(1) = msg->translation.y;
            x_(2) = msg->translation.z;
            x_.tail(3) = _angles;
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
        virtual void jointctrlcb(){
            // joint target
            sensor_msgs::JointState c_joint, t_joint;
            c_joint.position.resize(7);
            t_joint.position.resize(7);
            
            for (int i=0; i<7; i++){
                c_joint.position[i] = q_(i+2);
            }
            t_joint.position[0] = ui_.joint_t1->text().toFloat() * RAD;
            t_joint.position[1] = ui_.joint_t2->text().toFloat() * RAD;
            t_joint.position[2] = ui_.joint_t3->text().toFloat() * RAD;
            t_joint.position[3] = ui_.joint_t4->text().toFloat() * RAD;
            t_joint.position[4] = ui_.joint_t5->text().toFloat() * RAD;
            t_joint.position[5] = ui_.joint_t6->text().toFloat() * RAD;
            t_joint.position[6] = ui_.joint_t7->text().toFloat() * RAD;

            // joint kp, kv
            std::vector<double> kp, kv; 
             
            kp.push_back(ui_.joint_kp1->text().toFloat());
            kp.push_back(ui_.joint_kp2->text().toFloat());
            kp.push_back(ui_.joint_kp3->text().toFloat());
            kp.push_back(ui_.joint_kp4->text().toFloat());
            kp.push_back(ui_.joint_kp5->text().toFloat());
            kp.push_back(ui_.joint_kp6->text().toFloat());
            kp.push_back(ui_.joint_kp7->text().toFloat());

            if (ui_.ideal_kv->isChecked()){
                ui_.joint_kv1->setText(QString::number(2.0*std::sqrt(kp[0]), 'f', 3));
                ui_.joint_kv2->setText(QString::number(2.0*std::sqrt(kp[1]), 'f', 3));
                ui_.joint_kv3->setText(QString::number(2.0*std::sqrt(kp[2]), 'f', 3));
                ui_.joint_kv4->setText(QString::number(2.0*std::sqrt(kp[3]), 'f', 3));
                ui_.joint_kv5->setText(QString::number(2.0*std::sqrt(kp[4]), 'f', 3));
                ui_.joint_kv6->setText(QString::number(2.0*std::sqrt(kp[5]), 'f', 3));
                ui_.joint_kv7->setText(QString::number(2.0*std::sqrt(kp[6]), 'f', 3));                
            }

            kv.push_back(ui_.joint_kv1->text().toFloat());
            kv.push_back(ui_.joint_kv2->text().toFloat());
            kv.push_back(ui_.joint_kv3->text().toFloat());
            kv.push_back(ui_.joint_kv4->text().toFloat());
            kv.push_back(ui_.joint_kv5->text().toFloat());
            kv.push_back(ui_.joint_kv6->text().toFloat());
            kv.push_back(ui_.joint_kv7->text().toFloat());

            // joint masking
            std::vector<bool> mask;
            mask.push_back(ui_.joint_m1->isChecked());
            mask.push_back(ui_.joint_m2->isChecked());
            mask.push_back(ui_.joint_m3->isChecked());
            mask.push_back(ui_.joint_m4->isChecked());
            mask.push_back(ui_.joint_m5->isChecked());
            mask.push_back(ui_.joint_m6->isChecked());
            mask.push_back(ui_.joint_m7->isChecked());
            
            for (int i=0; i<7; i++)
                if (mask[i] == 0)
                    t_joint.position[i] = c_joint.position[i];

             
            plan_joint_srv_.request.current_joint = c_joint;
            plan_joint_srv_.request.target_joint.clear();
            plan_joint_srv_.request.target_joint.push_back(t_joint);
            plan_joint_srv_.request.traj_type = ui_.joint_trajectory_mode->currentIndex();
            plan_joint_srv_.request.kp = kp;
            plan_joint_srv_.request.kv = kv;
            
            plan_joint_srv_.request.mask.resize(7);
            for (int i=0; i<7; i++)
                plan_joint_srv_.request.mask[i].data = mask[i];

            string traj_type;
            if (ui_.joint_trajectory_mode->currentIndex() == 0){
                traj_type = "Constant";
                plan_joint_srv_.request.duration= 0.0;
                plan_joint_srv_.request.vel_limit = 0;
                plan_joint_srv_.request.acc_limit = 0;

            }
            else if (ui_.joint_trajectory_mode->currentIndex() == 1){
                traj_type = "Cubic Spline";
                plan_joint_srv_.request.duration= ui_.joint_trajectory_duration->text().toFloat();
                plan_joint_srv_.request.vel_limit = 0;
                plan_joint_srv_.request.acc_limit = 0;

            }
            else{
                traj_type = "Time Optimal Spline";
                plan_joint_srv_.request.duration= 0.0;
                if (target_q_vec_.size() > 0)
                    plan_joint_srv_.request.target_joint = target_q_vec_;
                plan_joint_srv_.request.vel_limit = ui_.joint_trajectory_tvel->text().toFloat();
                plan_joint_srv_.request.acc_limit = ui_.joint_trajectory_tacc->text().toFloat();

            }

            joint_plan_client_.call(plan_joint_srv_);
            ROS_WARN("%f", t_joint.position[0]);
            ROS_WARN("%d", ui_.joint_trajectory_mode->currentIndex());

            string log;
            log = "Time: " + to_string(time_) + "\n";
            log += "Trajectory Type: " + traj_type + "\n";
            
            if (ui_.joint_trajectory_mode->currentIndex() == 2){                
                for (int j=0; j<target_q_vec_.size(); j++){
                    log += "Desired Joint Targets: Waypoint #" + to_string(j) +"\t";
                    for (int i=0;i<7;i++)
                        log += to_string(target_q_vec_[j].position[i] *DEGREE) + "\t";
                }
            }
            else{
                log += "Desired Joint Target:\t";
                for (int i=0;i<7;i++)
                    log += to_string(t_joint.position[i] *DEGREE) + "\t";
            }
            ui_.joint_trajectory_log->appendPlainText(QString::fromStdString(log));
            target_q_vec_.clear();
        }
        virtual void jointctrlcb2(){
            int16_msg_.data = 900;
            custom_ctrl_pub_.publish(int16_msg_);
        }
        virtual void jointctrlcb3(){
            ui_.joint_t1->setText(QString::number(q_(2) * DEGREE, 'f', 3));
            ui_.joint_t2->setText(QString::number(q_(3) * DEGREE, 'f', 3));
            ui_.joint_t3->setText(QString::number(q_(4) * DEGREE, 'f', 3));
            ui_.joint_t4->setText(QString::number(q_(5) * DEGREE, 'f', 3));
            ui_.joint_t5->setText(QString::number(q_(6) * DEGREE, 'f', 3));
            ui_.joint_t6->setText(QString::number(q_(7) * DEGREE, 'f', 3));
            ui_.joint_t7->setText(QString::number(q_(8) * DEGREE, 'f', 3));
        }
        virtual void jointctrlcb4(){
            sensor_msgs::JointState t_joint;
            t_joint.position.resize(7);

            t_joint.position[0] = ui_.joint_t1->text().toFloat() * RAD;
            t_joint.position[1] = ui_.joint_t2->text().toFloat() * RAD;
            t_joint.position[2] = ui_.joint_t3->text().toFloat() * RAD;
            t_joint.position[3] = ui_.joint_t4->text().toFloat() * RAD;
            t_joint.position[4] = ui_.joint_t5->text().toFloat() * RAD;
            t_joint.position[5] = ui_.joint_t6->text().toFloat() * RAD;
            t_joint.position[6] = ui_.joint_t7->text().toFloat() * RAD;
            
            string log;
            log = "Add Waypoint:\t" ;

            for (int i=0;i<7;i++)
                log += to_string(t_joint.position[i] *DEGREE) + "\t";
            ui_.joint_trajectory_log->appendPlainText(QString::fromStdString(log));
            target_q_vec_.push_back(t_joint);


        }
        virtual void jointctrlcb5(){
            target_q_vec_.clear();
            string log;
            log = "Clear Waypoints" ;
            ui_.joint_trajectory_log->appendPlainText(QString::fromStdString(log));
        }
        virtual void basectrlcb1(){
            // base target
            geometry_msgs::Pose2D c_pose, t_pose;
                        
            plan_mobile_srv_.request.current_mobile_state.x = base_(0);
            plan_mobile_srv_.request.current_mobile_state.y = base_(1);
            plan_mobile_srv_.request.current_mobile_state.theta = base_(2);

            plan_mobile_srv_.request.target_mobile_pose.x = ui_.base_d1->text().toFloat();
            plan_mobile_srv_.request.target_mobile_pose.y = ui_.base_d2->text().toFloat();
            plan_mobile_srv_.request.target_mobile_pose.theta = ui_.base_d3->text().toFloat() * RAD;

            // obstacle
            obs_vec_.clear();
            int n_obs = ui_.obs_list->count(); 

            Eigen::VectorXd obs(4);
            for (int i=0; i< n_obs; i++){
                QListWidgetItem *item = ui_.obs_list->item(i);
                string obs_inf = item->text().toUtf8().constData();
                istringstream ss(obs_inf);
                string buffer;
                int j=0;
                while (getline(ss, buffer, ',')){
                    obs(j) = stod(buffer);
                    j++;
                }
                kimm_path_planner_ros_interface::Obstacle2D obs_2d;
                obs_2d.x1.data = obs(0);
                obs_2d.y1.data = obs(1);
                obs_2d.x2.data = obs(2);
                obs_2d.y2.data = obs(3);
                obs_vec_.push_back(obs_2d);
            }
            plan_mobile_srv_.request.Obstacles2D = obs_vec_;
            mobile_plan_client_.call(plan_mobile_srv_);
            
            
            string log;
            log = "Time: " + to_string(time_) + "\n";
            log += "Current Position: " + to_string(base_(0)) + " " + to_string(base_(1)) + " " + to_string(base_(2)) + "\n";
            log += "Target Position: " + to_string(ui_.base_d1->text().toFloat()) + " " + to_string(ui_.base_d2->text().toFloat()) + " " + to_string(ui_.base_d3->text().toFloat()) + "\n";
            if (plan_mobile_srv_.response.mobile_path.points.size() >= 1)
                log += "RRT based planner: Succeed\n";
            else{
                log += "RRT based planner: Failed\n";
            }
            
            ui_.base_log->appendPlainText(QString::fromStdString(log));

            //display
            visualization_msgs::MarkerArray response_list, req_list;
            int id = 0;

            visualization_msgs::Marker marker;
            marker.header.frame_id = "odom";
            marker.ns = "my_namespace";
            marker.pose.position.z = 0;
            
            marker.scale.x = 0.1;
            marker.scale.y = 0.015;
            marker.scale.z = 0.015;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            marker.type = visualization_msgs::Marker::ARROW;

            for (uint32_t i = 0; i < plan_mobile_srv_.response.mobile_path.points.size(); i=i+2)
            {
                marker.id = id;
                                
                marker.pose.position.x = plan_mobile_srv_.response.mobile_path.points[i].x;
                marker.pose.position.y = plan_mobile_srv_.response.mobile_path.points[i].y;

                double theta = plan_mobile_srv_.response.mobile_path.points[i].theta;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = sin(theta/2.);
                marker.pose.orientation.w = cos(theta/2.);
                
                id++;
                response_list.markers.push_back(marker);
            }

            base_traj_resp_pub_.publish(response_list);

            id = 0;
            marker.id = id;       
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;   
            marker.pose.position.x = plan_mobile_srv_.request.target_mobile_pose.x;
            marker.pose.position.y = plan_mobile_srv_.request.target_mobile_pose.y;

            double theta = plan_mobile_srv_.request.target_mobile_pose.theta;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = sin(theta/2.);
            marker.pose.orientation.w = cos(theta/2.);
            id++;
            req_list.markers.push_back(marker);
            
            marker.type = visualization_msgs::Marker::CUBE;
            for (int i=0; i< n_obs; i++){
                marker.id = id;  
                marker.pose.position.x = (obs_vec_[i].x1.data + obs_vec_[i].x2.data)/2.0;
                marker.pose.position.y = (obs_vec_[i].y1.data + obs_vec_[i].y2.data)/2.0;
                marker.pose.position.z = 0.2;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                marker.scale.x = abs(obs_vec_[i].x1.data - obs_vec_[i].x2.data);
                marker.scale.y = abs(obs_vec_[i].y1.data - obs_vec_[i].y2.data);
                marker.scale.z = 0.4;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                id++;
                req_list.markers.push_back(marker);
            }

            base_traj_req_pub_.publish(req_list);
        }
        virtual void basectrlcb2(){
            int16_msg_.data = 901;
            custom_ctrl_pub_.publish(int16_msg_);
        }
        virtual void basectrlcb3(){
            int16_msg_.data = 902;
            custom_ctrl_pub_.publish(int16_msg_);
        }
        virtual void basectrlcb4(){
            int16_msg_.data = 903;
            custom_ctrl_pub_.publish(int16_msg_);
        }
        virtual void basectrlcb5(){
            string obs_msg;

            obs_msg = to_string(ui_.obs_x1->text().toFloat());
            obs_msg += "," + to_string(ui_.obs_y1->text().toFloat());
            obs_msg += "," + to_string(ui_.obs_x2->text().toFloat());
            obs_msg += "," + to_string(ui_.obs_y2->text().toFloat());

            ui_.obs_list->addItem(obs_msg.c_str());
        }
        virtual void basectrlcb6(){
            QListWidgetItem *item = ui_.obs_list->takeItem(ui_.obs_list->currentIndex().row());
            ui_.obs_list->removeItemWidget(item);
        }
        virtual void se3ctrlcb1(){
            // se3 target
            geometry_msgs::Transform c_se3, t_se3;
            c_se3.translation.x = x_(0);
            c_se3.translation.y = x_(1);
            c_se3.translation.z = x_(2);

            Eigen::Quaternion<double> quat_tmp;
            Eigen::AngleAxisd rollAngle(x_(3), Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(x_(4), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(x_(5), Eigen::Vector3d::UnitZ());
            quat_tmp = yawAngle * pitchAngle * rollAngle;
            c_se3.rotation.x = quat_tmp.x();
            c_se3.rotation.y = quat_tmp.y();
            c_se3.rotation.z = quat_tmp.z();
            c_se3.rotation.w = quat_tmp.w();

            std::vector<bool> mask;
            mask.push_back(ui_.se3_m1->isChecked());
            mask.push_back(ui_.se3_m2->isChecked());
            mask.push_back(ui_.se3_m3->isChecked());
            mask.push_back(ui_.se3_m4->isChecked());
            mask.push_back(ui_.se3_m5->isChecked());
            mask.push_back(ui_.se3_m6->isChecked());

            if (ui_.use_rel->isChecked()){
                t_se3 = c_se3;
                if (mask[0] == 1)
                    t_se3.translation.x += ui_.se3_x->text().toFloat();
                if (mask[1] == 1)
                    t_se3.translation.y += ui_.se3_y->text().toFloat();
                if (mask[2] == 1)
                    t_se3.translation.z += ui_.se3_z->text().toFloat();
                if (mask[3] == 1)
                    rollAngle = Eigen::AngleAxisd(x_(3) + ui_.se3_roll->text().toFloat() * RAD, Eigen::Vector3d::UnitX());
                else
                    rollAngle = Eigen::AngleAxisd(x_(3), Eigen::Vector3d::UnitX());
                if (mask[4] == 1)
                    pitchAngle = Eigen::AngleAxisd(x_(4) + ui_.se3_pitch->text().toFloat() * RAD, Eigen::Vector3d::UnitY());
                else
                    pitchAngle = Eigen::AngleAxisd(x_(4), Eigen::Vector3d::UnitY());
                if (mask[5] == 1)
                    yawAngle = Eigen::AngleAxisd(x_(5) + ui_.se3_yaw->text().toFloat() * RAD, Eigen::Vector3d::UnitZ());
                else
                    yawAngle = Eigen::AngleAxisd(x_(5), Eigen::Vector3d::UnitZ());

                quat_tmp = yawAngle * pitchAngle * rollAngle;
                t_se3.rotation.x = quat_tmp.x();
                t_se3.rotation.y = quat_tmp.y();
                t_se3.rotation.z = quat_tmp.z();
                t_se3.rotation.w = quat_tmp.w();
            }
            else{
                t_se3 = c_se3;
                if (mask[0] == 1)
                    t_se3.translation.x = ui_.se3_x->text().toFloat();
                if (mask[1] == 1)
                    t_se3.translation.y = ui_.se3_y->text().toFloat();
                if (mask[2] == 1)
                    t_se3.translation.z = ui_.se3_z->text().toFloat();
                if (mask[3] == 1)
                    rollAngle = Eigen::AngleAxisd(ui_.se3_roll->text().toFloat() * RAD, Eigen::Vector3d::UnitX());
                else
                    rollAngle = Eigen::AngleAxisd(x_(3), Eigen::Vector3d::UnitX());
                if (mask[4] == 1)
                    pitchAngle = Eigen::AngleAxisd(ui_.se3_pitch->text().toFloat() * RAD, Eigen::Vector3d::UnitY());
                else
                    pitchAngle = Eigen::AngleAxisd(x_(4), Eigen::Vector3d::UnitY());
                if (mask[5] == 1)
                    yawAngle = Eigen::AngleAxisd(ui_.se3_yaw->text().toFloat() * RAD, Eigen::Vector3d::UnitZ());
                else
                    yawAngle = Eigen::AngleAxisd(x_(5), Eigen::Vector3d::UnitZ());

                quat_tmp = yawAngle * pitchAngle * rollAngle;
                t_se3.rotation.x = quat_tmp.x();
                t_se3.rotation.y = quat_tmp.y();
                t_se3.rotation.z = quat_tmp.z();
                t_se3.rotation.w = quat_tmp.w();
            }

            std::vector<double> kp, kv; 
            kp.push_back(ui_.se3_kp1->text().toFloat());
            kp.push_back(ui_.se3_kp2->text().toFloat());
            kp.push_back(ui_.se3_kp3->text().toFloat());
            kp.push_back(ui_.se3_kp4->text().toFloat());
            kp.push_back(ui_.se3_kp5->text().toFloat());
            kp.push_back(ui_.se3_kp6->text().toFloat());

            if (ui_.ideal_kv->isChecked()){
                ui_.se3_kv1->setText(QString::number(2.0*std::sqrt(kp[0]), 'f', 3));
                ui_.se3_kv2->setText(QString::number(2.0*std::sqrt(kp[1]), 'f', 3));
                ui_.se3_kv3->setText(QString::number(2.0*std::sqrt(kp[2]), 'f', 3));
                ui_.se3_kv4->setText(QString::number(2.0*std::sqrt(kp[3]), 'f', 3));
                ui_.se3_kv5->setText(QString::number(2.0*std::sqrt(kp[4]), 'f', 3));
                ui_.se3_kv6->setText(QString::number(2.0*std::sqrt(kp[5]), 'f', 3));             
            }          

            kv.push_back(ui_.se3_kv1->text().toFloat());
            kv.push_back(ui_.se3_kv2->text().toFloat());
            kv.push_back(ui_.se3_kv3->text().toFloat());
            kv.push_back(ui_.se3_kv4->text().toFloat());
            kv.push_back(ui_.se3_kv5->text().toFloat());
            kv.push_back(ui_.se3_kv6->text().toFloat());

            plan_se3_srv_.request.current_se3 = c_se3;
            plan_se3_srv_.request.target_se3.clear();
            plan_se3_srv_.request.target_se3.push_back(t_se3);
            plan_se3_srv_.request.traj_type = ui_.se3_trajectory_mode->currentIndex();
            plan_se3_srv_.request.iswholebody.data = ui_.wholebody_check->isChecked();
            plan_se3_srv_.request.kp = kp;
            plan_se3_srv_.request.kv = kv;
            
            plan_se3_srv_.request.mask.resize(6);
            for (int i=0; i<6; i++)
                plan_se3_srv_.request.mask[i].data = mask[i];

            string traj_type;
            if (ui_.se3_trajectory_mode->currentIndex() == 0){
                traj_type = "Constant";
                plan_se3_srv_.request.duration= 0.0;
                plan_se3_srv_.request.vel_limit = 0;
                plan_se3_srv_.request.acc_limit = 0;

            }
            else if (ui_.se3_trajectory_mode->currentIndex() == 1){
                traj_type = "Cubic Spline";
                plan_se3_srv_.request.duration= ui_.se3_trajectory_duration->text().toFloat();
                plan_se3_srv_.request.vel_limit = 0;
                plan_se3_srv_.request.acc_limit = 0;

            }
            else{
                traj_type = "Time Optimal Spline";
                plan_se3_srv_.request.duration= 0.0;
                if (target_x_vec_.size() > 0)
                    plan_se3_srv_.request.target_se3 = target_x_vec_;
                plan_se3_srv_.request.vel_limit = ui_.se3_trajectory_tvel->text().toFloat();
                plan_se3_srv_.request.acc_limit = ui_.se3_trajectory_tvel->text().toFloat();
            }

            se3_plan_client_.call(plan_se3_srv_);
            
            string log;
            log = "Time: " + to_string(time_) + "\n";
            log += "Trajectory Type: " + traj_type + "\n";
            
            if (ui_.se3_trajectory_mode->currentIndex() == 2){                
                for (int j=0; j<target_x_vec_.size(); j++){
                    log += "Desired SE3 Targets: Waypoint #" + to_string(j) +"\t";
                    log += to_string(target_x_vec_[j].translation.x) + "\t" + to_string(target_x_vec_[j].translation.y) + "\t" +to_string(target_x_vec_[j].translation.z) + "\t" ;
                }
            }
            else{
                log += "Desired SE3 Target:\t";
                log += to_string(t_se3.translation.x) + "\t" + to_string(t_se3.translation.y) + "\t" +to_string(t_se3.translation.z) 
                       + "\t" + to_string(t_se3.rotation.x) + "\t" + to_string(t_se3.rotation.y) + "\t" +to_string(t_se3.rotation.z) + "\t"   + to_string(t_se3.rotation.w) + "\t"   ;
            }
            ui_.se3_trajectory_log->appendPlainText(QString::fromStdString(log));
            target_x_vec_.clear();

            //display
            visualization_msgs::MarkerArray response_list;
            int id = 0;

            visualization_msgs::Marker marker;
            marker.header.frame_id = "odom";
            marker.ns = "my_namespace";
            marker.pose.position.z = 0;
            
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.scale.z = 0.01;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            marker.type = visualization_msgs::Marker::SPHERE;

            for (uint32_t i = 0; i < plan_se3_srv_.response.res_traj.size(); i=i+10)
            {
                marker.id = id;
                                
                marker.pose.position.x = plan_se3_srv_.response.res_traj[i].translation.x;
                marker.pose.position.y = plan_se3_srv_.response.res_traj[i].translation.y;
                marker.pose.position.z = plan_se3_srv_.response.res_traj[i].translation.z + 0.25;

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                
                id++;
                response_list.markers.push_back(marker);
            }

            ee_traj_resp_pub_.publish(response_list);

        }
        virtual void se3ctrlcb2(){
            int16_msg_.data = 904;
            custom_ctrl_pub_.publish(int16_msg_);
        }
        virtual void se3ctrlcb3(){
            ui_.se3_x->setText(QString::number(x_(0), 'f', 3));
            ui_.se3_y->setText(QString::number(x_(1), 'f', 3));
            ui_.se3_z->setText(QString::number(x_(2), 'f', 3));
            ui_.se3_roll->setText(QString::number(x_(3) * DEGREE, 'f', 3));
            ui_.se3_pitch->setText(QString::number(x_(4) * DEGREE, 'f', 3));
            ui_.se3_yaw->setText(QString::number(x_(5) * DEGREE, 'f', 3));
        }
        virtual void se3ctrlcb4(){
            geometry_msgs::Transform t_x;
            t_x.translation.x = ui_.se3_x->text().toFloat();
            t_x.translation.y = ui_.se3_y->text().toFloat();
            t_x.translation.z = ui_.se3_z->text().toFloat();

            Eigen::Quaternion<double> quat_tmp;
            Eigen::AngleAxisd rollAngle(x_(3), Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(x_(4), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(x_(5), Eigen::Vector3d::UnitZ());
            quat_tmp = yawAngle * pitchAngle * rollAngle;
            
            t_x.rotation.x = quat_tmp.x();
            t_x.rotation.y = quat_tmp.y();
            t_x.rotation.z = quat_tmp.z();
            t_x.rotation.w = quat_tmp.w();
                        
            string log;
            log = "Add Waypoint:\t" ;
            log += to_string(t_x.translation.x) + "\t" + to_string(t_x.translation.y) + "\t" +to_string(t_x.translation.z) ;
            ui_.se3_trajectory_log->appendPlainText(QString::fromStdString(log));
            target_x_vec_.push_back(t_x);
        }
        virtual void se3ctrlcb5(){
            target_x_vec_.clear();
            string log;
            log = "Clear Waypoints" ;
            ui_.se3_trajectory_log->appendPlainText(QString::fromStdString(log));
        }

    public:
        ros::Publisher run_pub_, quit_pub_, custom_ctrl_pub_, base_traj_resp_pub_, base_traj_req_pub_, obs_pub_, ee_traj_resp_pub_;
        ros::Subscriber simtime_sub_, jointstate_sub_, torquestate_sub_, base_state_sub_, ee_state_sub_;
        ros::ServiceClient joint_plan_client_, mobile_plan_client_, se3_plan_client_;

        kimm_path_planner_ros_interface::plan_mobile_path plan_mobile_srv_;
        kimm_se3_planner_ros_interface::plan_se3_path plan_se3_srv_;
        kimm_joint_planner_ros_interface::plan_joint_path plan_joint_srv_;


        Eigen::VectorXd q_, x_;
        Eigen::Vector3d base_;
        Eigen::MatrixXd rot_;
        std::vector<sensor_msgs::JointState> target_q_vec_;
        std::vector<geometry_msgs::Transform>  target_x_vec_;
        std::vector<kimm_path_planner_ros_interface::Obstacle2D> obs_vec_;
        bool issimulation_;

        


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
