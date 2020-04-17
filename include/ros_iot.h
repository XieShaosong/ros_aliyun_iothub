#include "ros/ros.h"
#include "ros_aliyun_iothub/status.h"
#include "ros_aliyun_iothub/control.h"

#include "infra_compat.h"
#include "dev_sign_api.h"
#include "mqtt_api.h"
#include "wrappers.h"

using namespace std;

namespace ros_iot
{
class AliIot
{
public:
    AliIot();

    ros::NodeHandle n;
    ros::Timer timer;
    ros::Subscriber status_sub;
    ros::ServiceClient ctrl_client;
    static string cmd;
    void intervalCallback(const ros::TimerEvent &);
    ros_aliyun_iothub::status demo_status;
    void statusCallback(const ros_aliyun_iothub::status::ConstPtr &msg);
    void handleCommand(const char *cmd);
public:
    char product_key[IOTX_PRODUCT_KEY_LEN + 1] = "a1KVWijCHZx";
    char product_secret[IOTX_PRODUCT_SECRET_LEN + 1] = "h4I4dneEFp7EImTv";
    char device_name[IOTX_DEVICE_NAME_LEN + 1] = "cq_iot_test";
    char device_secret[IOTX_DEVICE_SECRET_LEN + 1] = "yJ7TTfwROgGCoMd2WMmpOKc9TG92DR7B";

    void *pclient = NULL;
    iotx_mqtt_param_t mqtt_params;
    void iot_init();
    bool iot_publish(void *handle);
    bool iot_subscribe(void *handle);
    static void iot_message_arrive(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg);
    static void iot_event_handle(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg);
    static int iot_everything_state_handle(const int state_code, const char *state_message);
    static int iot_identity_response_handle(const char *payload);
};
}
