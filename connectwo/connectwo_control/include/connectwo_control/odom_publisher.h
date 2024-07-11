#ifndef ODOM_PUBLISHER_H
#define ODOM_PUBLISHER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h> // IMU 메시지 타입 포함

class OdomPublisher
{
public:
    OdomPublisher();
    ~OdomPublisher();

    void publishOdometry();

private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg); // 추가: IMU 콜백 함수

    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber imu_sub_; // 추가: IMU 구독자
    
    tf::TransformBroadcaster odom_broadcaster_;

    double x_, y_, z_, th_;
    double vx_, vy_, vz_, vth_;

    double roll_, pitch_; // 추가: roll과 pitch 멤버 변수

    ros::Time current_time_, last_time_;

    
    // 추가: pos_, vel_, eff_ 배열 멤버 변수 선언, 개수는 wheel의 개수만큼 지정
    double pos_[2];
    double vel_[2];
    double eff_[2];
};

#endif // ODOM_PUBLISHER_H