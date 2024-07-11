#ifndef CMD_VEL_AMPLIFIER_H
#define CMD_VEL_AMPLIFIER_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class CmdVelAmplifier
{
public:
    CmdVelAmplifier();
    ~CmdVelAmplifier();

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_subscriber_;
    ros::Publisher cmd_vel_publisher_;

    void amplifyTwist(geometry_msgs::Twist& msg);
};

#endif // CMD_VEL_AMPLIFIER_H