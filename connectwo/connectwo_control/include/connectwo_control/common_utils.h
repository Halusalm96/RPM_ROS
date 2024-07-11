#ifndef COMMON_UTILS_H
#define COMMON_UTILS_H

#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <vector>
#include <string>

void updateJointStates(const sensor_msgs::JointState::ConstPtr& msg, const std::vector<std::string>& joint_names, double* pos, double* vel, double* eff) {
    if (msg->name.size() != joint_names.size() || msg->position.size() != joint_names.size() || msg->velocity.size() != joint_names.size() || msg->effort.size() != joint_names.size()) {
        ROS_WARN("JointState message has unexpected size");
        return;
    }
    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (size_t j = 0; j < joint_names.size(); ++j) {
            if (msg->name[i] == joint_names[j]) {
                pos[j] = msg->position[i];
                vel[j] = msg->velocity[i];
                eff[j] = msg->effort[i];
                break;
            }
        }
    }
}


#endif // COMMON_UTILS_H