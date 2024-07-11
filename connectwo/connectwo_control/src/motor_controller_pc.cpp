#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class MotorControllerPC {
public:
    MotorControllerPC(ros::NodeHandle& nh) 
        : nh_(nh) {
        joint_state_sub_ = nh_.subscribe("joint_states", 10, &MotorControllerPC::jointStateCallback, this);
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        // 여기에서 joint_states 데이터를 처리
        ROS_INFO_STREAM("Received joint states: " << *msg);
    }

private:
    ros::NodeHandle& nh_;
    ros::Subscriber joint_state_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_controller_pc");
    ros::NodeHandle nh;

    MotorControllerPC motor_controller(nh);

    ros::spin();

    return 0;
}