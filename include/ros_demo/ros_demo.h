#include "ros/ros.h"
#include "ros_aliyun_iothub/status.h"
#include "ros_aliyun_iothub/control.h"

using namespace std;

namespace ros_demo
{
class Demo
{
public:
    Demo();

    ros::NodeHandle n;
    ros::Timer timer;
    ros::Publisher status_pub;
    ros::ServiceServer ctrl_srv;
    void intervalCallback(const ros::TimerEvent &);
    bool commandCallback(ros_aliyun_iothub::control::Request &req, ros_aliyun_iothub::control::Response &res);
};
}