#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class MotorControllerRPI {
public:
    MotorControllerRPI(ros::NodeHandle& nh)
        : nh_(nh) {
        joint_state_sub_ = nh_.subscribe("joint_states", 10, &MotorControllerRPI::jointStateCallback, this);
        ROS_INFO("MotorControllerRPI initialized and subscribed to joint_states");
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        // joint_states 데이터를 처리합니다.
        ROS_INFO_STREAM("Received joint states: " << *msg);
        for (size_t i = 0; i < msg->name.size(); ++i) {
            ROS_INFO("Joint: %s, Position: %f, Velocity: %f, Effort: %f",
                     msg->name[i].c_str(),
                     msg->position[i],
                     msg->velocity[i],
                     msg->effort[i]);
        }
    }

private:
    ros::NodeHandle& nh_;
    ros::Subscriber joint_state_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_controller_rpi");
    ros::NodeHandle nh;

    ROS_INFO("Starting MotorControllerRPI node");
    MotorControllerRPI motor_controller(nh);

    ros::spin();

    return 0;
}
