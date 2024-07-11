#ifndef CONNECTWO_CONTROL_H
#define CONNECTWO_CONTROL_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include "common_utils.h"

class ConnectwoControl : public hardware_interface::RobotHW
{
public:
    ConnectwoControl(ros::NodeHandle &nh) :
    joint_names_{"wheel_right_joint", "wheel_left_joint", "caster_back_right_joint", "caster_back_left_joint"},
    cmd_{0.0, 0.0, 0.0, 0.0},
    pos_{0.0, 0.0, 0.0, 0.0},
    vel_{0.0, 0.0, 0.0, 0.0},
    eff_{0.0, 0.0, 0.0, 0.0}
    {
        // Joint state interface
        hardware_interface::JointStateHandle state_handle_front_right("wheel_right_joint", &pos_[0], &vel_[0], &eff_[0]);
        jnt_state_interface_.registerHandle(state_handle_front_right);
        hardware_interface::JointStateHandle state_handle_front_left("wheel_left_joint", &pos_[1], &vel_[1], &eff_[1]);
        jnt_state_interface_.registerHandle(state_handle_front_left);
        hardware_interface::JointStateHandle state_handle_back_right("caster_back_right_joint", &pos_[2], &vel_[2], &eff_[2]);
        jnt_state_interface_.registerHandle(state_handle_back_right);
        hardware_interface::JointStateHandle state_handle_back_left("caster_back_left_joint", &pos_[3], &vel_[3], &eff_[3]);
        jnt_state_interface_.registerHandle(state_handle_back_left);

        registerInterface(&jnt_state_interface_);

        // Joint position interface
        hardware_interface::JointHandle pos_handle_front_right(jnt_state_interface_.getHandle("wheel_right_joint"), &cmd_[0]);
        jnt_pos_interface_.registerHandle(pos_handle_front_right);
        hardware_interface::JointHandle pos_handle_front_left(jnt_state_interface_.getHandle("wheel_left_joint"), &cmd_[1]);
        jnt_pos_interface_.registerHandle(pos_handle_front_left);
        hardware_interface::JointHandle pos_handle_back_right(jnt_state_interface_.getHandle("caster_back_right_joint"), &cmd_[2]);
        jnt_pos_interface_.registerHandle(pos_handle_back_right);
        hardware_interface::JointHandle pos_handle_back_left(jnt_state_interface_.getHandle("caster_back_left_joint"), &cmd_[3]);
        jnt_pos_interface_.registerHandle(pos_handle_back_left);

        registerInterface(&jnt_pos_interface_);

        // 퍼블리셔와 서브스크라이버 초기화
        joint_state_sub_ = nh.subscribe("/joint_states", 10, &ConnectwoControl::jointStateCallback, this);
        front_right_wheel_pub_ = nh.advertise<std_msgs::Float64>("/front_right_wheel_position_controller/command", 10);
        front_left_wheel_pub_ = nh.advertise<std_msgs::Float64>("/front_left_wheel_position_controller/command", 10);
        caster_right_wheel_pub_ = nh.advertise<std_msgs::Float64>("/caster_right_wheel_position_controller/command", 10);
        caster_left_wheel_pub_ = nh.advertise<std_msgs::Float64>("/caster_left_wheel_position_controller/command", 10);
    }
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        updateJointStates(msg, joint_names_, pos_, vel_, eff_);
    }

    void read() {
        // 이미 jointStateCallback에서 joint_position_, joint_velocity_, joint_effort_ 값을 업데이트
    }

    void write() {
        // 명령 값을 하드웨어로 쓰는 코드
        std_msgs::Float64 cmd;
        cmd.data = cmd_[0];
        front_right_wheel_pub_.publish(cmd);
        cmd.data = cmd_[1];
        front_left_wheel_pub_.publish(cmd);
        cmd.data = cmd_[2];
        caster_right_wheel_pub_.publish(cmd);
        cmd.data = cmd_[3];
        caster_left_wheel_pub_.publish(cmd);
    }

private:
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_pos_interface_;
    ros::Subscriber joint_state_sub_;
    ros::Publisher front_right_wheel_pub_;
    ros::Publisher front_left_wheel_pub_;
    ros::Publisher caster_right_wheel_pub_;
    ros::Publisher caster_left_wheel_pub_;
    double cmd_[4];
    double pos_[4];
    double vel_[4];
    double eff_[4];
    std::vector<std::string> joint_names_;  // 조인트 이름을 저장하는 벡터
};

#endif // CONNECTWO_CONTROL_H