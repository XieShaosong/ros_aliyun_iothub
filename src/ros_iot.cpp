#include "ros_iot.h"
using namespace ros_iot;

string AliIot::cmd = "";

AliIot::AliIot()
{
    iot_init();
    timer = n.createTimer(ros::Duration(1), &AliIot::intervalCallback, this);
    status_sub = n.subscribe("demo_status", 1, &AliIot::statusCallback, this);
    ctrl_client = n.serviceClient<ros_aliyun_iothub::control>("demo_control");
}

void AliIot::iot_init()
{
    IOT_Ioctl(IOTX_IOCTL_SET_PRODUCT_KEY, product_key);
    IOT_Ioctl(IOTX_IOCTL_SET_DEVICE_NAME, device_name);
    IOT_Ioctl(IOTX_IOCTL_SET_DEVICE_SECRET, device_secret);

    IOT_RegisterCallback(ITE_IDENTITY_RESPONSE, iot_identity_response_handle);
    IOT_RegisterCallback(ITE_STATE_EVERYTHING, iot_everything_state_handle);

    memset(&mqtt_params, 0x0, sizeof(mqtt_params));

    mqtt_params.handle_event.h_fp = iot_event_handle;

    pclient = IOT_MQTT_Construct(&mqtt_params);
    if (NULL == pclient) 
    {
        ROS_INFO("MQTT construct failed");
        return;
    }

    IOT_Ioctl(IOTX_IOCTL_GET_PRODUCT_KEY, product_key);
    IOT_Ioctl(IOTX_IOCTL_GET_DEVICE_NAME, device_name);

    if (!iot_subscribe(pclient))
    {
        IOT_MQTT_Destroy(&pclient);
        ROS_INFO("iot subscribe failed");
        return;
    }
}

void AliIot::iot_message_arrive(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg)
{
    iotx_mqtt_topic_info_t *topic_info = (iotx_mqtt_topic_info_pt)msg->msg;
    
    switch (msg->event_type)
    {
        case IOTX_MQTT_EVENT_PUBLISH_RECEIVED:
            ROS_INFO("Message Arrived:");
            ROS_INFO("Payload: %.*s", topic_info->payload_len, topic_info->payload);
            cmd = topic_info->payload;
            break;

        default:
            break;
    }
}

bool AliIot::iot_subscribe(void *handle)
{
    int res = 0;
    const char *fmt = "/%s/%s/user/get";
    char *topic = NULL;
    int topic_len = 0;

    topic_len = strlen(fmt) + strlen(product_key) + strlen(device_name) + 1;
    topic = HAL_Malloc(topic_len);
    if (topic == NULL)
    {
        ROS_INFO("memory not enough");
        return false;
    }
    memset(topic, 0, topic_len);
    HAL_Snprintf(topic, topic_len, fmt, product_key, device_name);

    res = IOT_MQTT_Subscribe(handle, topic, IOTX_MQTT_QOS0, iot_message_arrive, NULL);
    if (res < 0)
    {
        ROS_INFO("subscribe failed");
        HAL_Free(topic);
        return false;
    }

    HAL_Free(topic);
    return true;
}

bool AliIot::iot_publish(void *handle)
{
    int res = 0;
    const char *fmt= "/%s/%s/user/update";
    char *topic = NULL;
    int topic_len = 0;
    string payload = "{\"status\":\"" + demo_status.status + "\",\"target_value\":\"" + to_string(demo_status.target_value) +"\",\"current_value\":\"" + to_string(demo_status.current_value) +"\"}";

    topic_len = strlen(fmt)+strlen(product_key)+strlen(device_name)+1;
    topic = HAL_Malloc(topic_len);
    if (topic == NULL)
    {
        ROS_INFO("memory not enough");
        return false;
    }
    memset(topic, 0, topic_len);
    HAL_Snprintf(topic, topic_len, fmt, product_key, device_name);

    res = IOT_MQTT_Publish_Simple(0, topic, IOTX_MQTT_QOS0, payload.c_str(), strlen(payload.c_str()));
    if (res < 0)
    {
        ROS_INFO("publish failed, res = %d", res);
        HAL_Free(topic);
        return false;
    }

    HAL_Free(topic);
    return true;
}

void AliIot::iot_event_handle(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg)
{
    ROS_INFO("msg->event_type: %d", msg->event_type);
}

int AliIot::iot_everything_state_handle(const int state_code, const char *state_message)
{
    ROS_INFO("recv -0x%04dX(%s)", -state_code, state_message);
    return 0;
}

int AliIot::iot_identity_response_handle(const char *payload)
{
    ROS_INFO("identify: %s", payload);
    return 0;
}

void AliIot::intervalCallback(const ros::TimerEvent &)
{
    iot_publish(pclient);
    IOT_MQTT_Yield(pclient, 200);

    if (cmd != "")
    {
        handleCommand(cmd.c_str());
        cmd = "";
    }
}

void AliIot::statusCallback(const ros_aliyun_iothub::status::ConstPtr &msg)
{
    demo_status = *msg;
}

void AliIot::handleCommand(const char *msg)
{
    ros_aliyun_iothub::control ctrl;
    ctrl.request.cmd = msg;
    ctrl_client.call(ctrl);
}
