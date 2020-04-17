#include "ros_demo/ros_demo.h"

using namespace ros_demo;

Demo::Demo()
{
    status_pub = n.advertise<ros_aliyun_iothub::status>("demo_status", 1);
    ctrl_srv = n.advertiseService("demo_control", &Demo::commandCallback, this);
    timer = n.createTimer(ros::Duration(1), &Demo::intervalCallback, this);
}

void Demo::intervalCallback(const ros::TimerEvent &)
{
    ros_aliyun_iothub::status demo_status;
    demo_status.status = "running";
    demo_status.target_value = 2.0;
    demo_status.current_value = 1.7;
    status_pub.publish(demo_status);
}

bool Demo::commandCallback(ros_aliyun_iothub::control::Request &req, ros_aliyun_iothub::control::Response &res)
{
    ROS_INFO("%s", req.cmd.c_str());
    res.result = "success";
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_demo");
    ROS_INFO("ros demo started");

    Demo demo;

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}