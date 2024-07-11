#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

ros::Publisher cmd_vel_publisher;

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Copy the received message to modify it
    geometry_msgs::Twist amplified_cmd = *msg;

    // Modify the angular.z value based on conditions
    if (msg->linear.x == 0.0 && msg->angular.z > 0 && msg->angular.z <= 2.6)
    {
        amplified_cmd.angular.z = 2.7;
    }
    else if (msg->linear.x == 0.0 && msg->angular.z >= -2.6 && msg->angular.z < 0)
    {
        amplified_cmd.angular.z = -2.7;
    }
    else if (msg->linear.x > 0 && msg->linear.x < 0.05)
    {
        amplified_cmd.linear.x *= 1.3;
        if (msg->angular.z > 0 && msg->angular.z <= 2.6)
        {
            amplified_cmd.angular.z = 2.7;
        }
        else if (msg->angular.z >= -2.6 && msg->angular.z < 0)
        {
            amplified_cmd.angular.z = -2.7;
        }

        if (amplified_cmd.angular.z > 3.0)
        {
            amplified_cmd.angular.z = 2.7;
        }
        else if (amplified_cmd.angular.z < -3.0)
        {
            amplified_cmd.angular.z = -2.7;
        }
    }
    else
    {
        amplified_cmd.linear.x *= 1.3;
        amplified_cmd.angular.z *= 2.7;
        amplified_cmd.angular.z = round(amplified_cmd.angular.z * 100) / 100; // rounding to 2 decimal places

        if (amplified_cmd.angular.z > 3.0)
        {
            amplified_cmd.angular.z = 2.7;
        }
        else if (amplified_cmd.angular.z < -3.0)
        {
            amplified_cmd.angular.z = -2.7;
        }
    }

    // Publish the amplified message to cmd_vel topic
    cmd_vel_publisher.publish(amplified_cmd);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_vel_amplifier");
    ros::NodeHandle nh;

    // Subscriber to cmd_vel_amplifier topic
    ros::Subscriber cmd_vel_subscriber = nh.subscribe("cmd_vel_amplifier", 10, cmd_vel_callback);

    // Publisher to cmd_vel topic
    cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Spin
    ros::spin();

    return 0;
}
