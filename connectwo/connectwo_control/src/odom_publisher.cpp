#include "odom_publisher.h"
#include "common_utils.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

OdomPublisher::OdomPublisher() :
    x_(0.0), y_(0.0), z_(0.0), th_(0.0), vx_(0.0), vy_(0.0), vz_(0.0), vth_(0.0), roll_(0.0), pitch_(0.0)
{
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 50, &OdomPublisher::cmdVelCallback, this);
    joint_state_sub_ = nh_.subscribe("joint_states", 50, &OdomPublisher::jointStatesCallback, this);
    imu_sub_ = nh_.subscribe("imu", 50, &OdomPublisher::imuCallback, this); // IMU 데이터 구독
    current_time_ = ros::Time::now();
    last_time_ = ros::Time::now();

    // 배열 초기화   
    std::fill(std::begin(pos_), std::end(pos_), 0.0);
    std::fill(std::begin(vel_), std::end(vel_), 0.0);
    std::fill(std::begin(eff_), std::end(eff_), 0.0);
}

OdomPublisher::~OdomPublisher() {}

void OdomPublisher::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    vth_ = msg->angular.z;
    //ROS_INFO("cmdVelCallback: vx=%f, vy=%f, vth=%f", vx_, vy_, vth_);
}

void OdomPublisher::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // std::vector<std::string> joint_names = {"wheel_right_joint", "wheel_left_joint", "caster_back_right_joint", "caster_back_left_joint"};
    std::vector<std::string> joint_names = {"wheel_right_joint", "wheel_left_joint"};
    // 메시지의 크기가 예상과 다른 경우 경고 메시지 출력
    if (msg->name.size() != joint_names.size() || 
        msg->position.size() != joint_names.size() || 
        msg->velocity.size() != joint_names.size() || 
        msg->effort.size() != joint_names.size()) {
        ROS_WARN("JointState message has unexpected size");
        return;
    }

    updateJointStates(msg, joint_names, pos_, vel_, eff_);


    double right_wheel_velocity = vel_[0];
    double left_wheel_velocity = vel_[1];

    //double caster_back_right_velocity = vel_[2];
    //double caster_back_left_velocity = vel_[3];

    double wheel_radius = 0.125;
    double wheel_base = 0.37;

    double v_left = left_wheel_velocity * wheel_radius;
    double v_right = right_wheel_velocity * wheel_radius;
    //double v_caster_back_right = caster_back_right_velocity * wheel_radius;
    //double v_caster_back_left = caster_back_left_velocity * wheel_radius;

    // vx_ = (v_right + v_left + v_caster_back_right + v_caster_back_left) / 4.0;
    // vy_ = (v_right - v_left - v_caster_back_right + v_caster_back_left) / 4.0;
    // vth_ = ((v_right - v_left) + (v_caster_back_right - v_caster_back_left)) / (2.0 * wheel_base);

    vx_ = (v_right + v_left) / 2.0;
    vy_ = 0.0;  // 좌우 바퀴만 사용할 경우 y 방향 속도는 0
    vth_ = (v_right - v_left) / wheel_base;
    //ROS_INFO("jointStatesCallback: vx=%f, vy=%f, vth=%f", vx_, vy_, vth_);
}

void OdomPublisher::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll_, pitch_, th_); // roll_, pitch_, th_ 업데이트
}

void OdomPublisher::publishOdometry()
{
    //ros::spinOnce();
    current_time_ = ros::Time::now();
    double dt = (current_time_ - last_time_).toSec();

    double delta_x = (vx_ * cos(th_) - vy_ * sin(th_)) * dt;
    double delta_y = (vx_ * sin(th_) + vy_ * cos(th_)) * dt;
    //double delta_z = vz_ * dt; // z축 이동 반영
    double delta_th = vth_ * dt;

    x_ += delta_x;
    y_ += delta_y;
    //z_ += delta_z; // z 업데이트
    th_ += delta_th;

    //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, th_);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = z_;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster_.sendTransform(odom_trans);


    nav_msgs::Odometry odom;
    odom.header.stamp = current_time_;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = z_;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.linear.y = vy_;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.z = vth_;
    odom_pub_.publish(odom);

    last_time_ = current_time_;
    //ROS_INFO("publishOdometry: x=%f, y=%f, th=%f", x_, y_, th_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_publisher");
    OdomPublisher odom_publisher;
    ros::Rate r(50.0);
    while (ros::ok())
    {
        odom_publisher.publishOdometry();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}