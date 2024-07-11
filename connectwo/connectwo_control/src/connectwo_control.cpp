#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "connectwo_control.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "connectwo_control");
    ros::NodeHandle nh;

    // 하드웨어 인터페이스 객체 생성
    ConnectwoControl connectwo_control(nh);
    controller_manager::ControllerManager cm(&connectwo_control, nh);

    ros::Rate rate(50.0);  // 50 Hz

    while (ros::ok())
    {
        ros::Duration elapsed_time = rate.expectedCycleTime();
        connectwo_control.read();
        cm.update(ros::Time::now(), elapsed_time);
        connectwo_control.write();
        rate.sleep();
    }

    return 0;
}