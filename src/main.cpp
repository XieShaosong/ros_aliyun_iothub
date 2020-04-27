#include "ros_iot.h"
using namespace ros_iot;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_iot");
  ROS_INFO("ros iot started");

  AliIot iot;

  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();

  IOT_MQTT_Destroy(&iot.pclient);
  IOT_Linkkit_Close(iot.g_user_iot_ctx.master_devid);
  ROS_INFO("close");
  return 0;
}