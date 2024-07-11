#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle nh;

    tf::TransformBroadcaster br;
    tf::Transform transform;
    ros::Rate rate(10.0);  // 10 Hz 주기

    while (ros::ok()){
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));

        // 현재 시간으로 타임스템프 설정
        ros::Time current_time = ros::Time::now();

        // 변환 송신
        br.sendTransform(tf::StampedTransform(transform, current_time, "map", "odom"));

        rate.sleep();
    }

    return 0;
}
