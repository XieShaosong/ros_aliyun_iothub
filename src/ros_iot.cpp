#include "ros_iot.h"
using namespace ros_iot;

string AliIot::cmd = "";
AliIot::iot_ctx_t AliIot::g_user_iot_ctx;

AliIot::AliIot()
{
    iot_init();
    topic_timer = n.createTimer(ros::Duration(1), &AliIot::intervalTopicCallback, this);
    linkkit_timer = n.createTimer(ros::Duration(5), &AliIot::intervalLinkkitCallback, this);
    timer = n.createTimer(ros::Duration(0.2), &AliIot::intervalCallback, this);
    status_sub = n.subscribe("demo_status", 1, &AliIot::statusCallback, this);
    ctrl_client = n.serviceClient<ros_aliyun_iothub::control>("demo_control");
}

void AliIot::iot_init()
{
    int res = 0;
    int cnt = 0;
    iotx_linkkit_dev_meta_info_t master_meta_info;
    int dynamic_register = 0, post_reply_need = 0;
    memset(&g_user_iot_ctx, 0, sizeof(iot_ctx_t));

    memset(&master_meta_info, 0, sizeof(iotx_linkkit_dev_meta_info_t));
    memcpy(master_meta_info.product_key, product_key, strlen(product_key));
    memcpy(master_meta_info.product_secret, product_secret, strlen(product_secret));
    memcpy(master_meta_info.device_name, device_name, strlen(device_name));
    memcpy(master_meta_info.device_secret, device_secret, strlen(device_secret));

    IOT_SetLogLevel(IOT_LOG_WARNING);

    IOT_Ioctl(IOTX_IOCTL_SET_PRODUCT_KEY, product_key);
    IOT_Ioctl(IOTX_IOCTL_SET_DEVICE_NAME, device_name);
    IOT_Ioctl(IOTX_IOCTL_SET_DEVICE_SECRET, device_secret);

    // IOT_RegisterCallback(ITE_IDENTITY_RESPONSE, iot_identity_response_handle);
    // IOT_RegisterCallback(ITE_STATE_EVERYTHING, iot_everything_state_handle);
    // IOT_RegisterCallback(ITE_STATE_EVERYTHING, iot_sdk_state_dump);
    IOT_RegisterCallback(ITE_CONNECT_SUCC, iot_connected_event_handler);
    IOT_RegisterCallback(ITE_DISCONNECTED, iot_disconnected_event_handler);
    IOT_RegisterCallback(ITE_SERVICE_REQUEST, iot_service_request_event_handler);
    IOT_RegisterCallback(ITE_PROPERTY_SET, iot_property_set_event_handler);
    // IOT_RegisterCallback(ITE_REPORT_REPLY, iot_report_reply_event_handler);
    IOT_RegisterCallback(ITE_TRIGGER_EVENT_REPLY, iot_trigger_event_reply_event_handler);
    IOT_RegisterCallback(ITE_TIMESTAMP_REPLY, iot_timestamp_reply_event_handler);
    IOT_RegisterCallback(ITE_INITIALIZE_COMPLETED, iot_initialized);
    IOT_RegisterCallback(ITE_FOTA, iot_fota_event_handler);
    IOT_RegisterCallback(ITE_CLOUD_ERROR, iot_cloud_error_handler);
    IOT_RegisterCallback(ITE_DYNREG_DEVICE_SECRET, iot_dynreg_device_secret);

    dynamic_register = 0;
    IOT_Ioctl(IOTX_IOCTL_SET_DYNAMIC_REGISTER, (void *)&dynamic_register);

    post_reply_need = 1;
    IOT_Ioctl(IOTX_IOCTL_RECV_EVENT_REPLY, (void *)&post_reply_need);

    do 
    {
        g_user_iot_ctx.master_devid = IOT_Linkkit_Open(IOTX_LINKKIT_DEV_TYPE_MASTER, &master_meta_info);
        if (g_user_iot_ctx.master_devid >= 0)
        {
            break;
        }
        ROS_INFO("IOT_Linkkit_Open failed! retry after 2s");
        sleep(2);
    } while (1);

    do
    {
        res = IOT_Linkkit_Connect(g_user_iot_ctx.master_devid);
        if (res >=0)
        {
            break;
        }
        ROS_INFO("IOT_Linkkit_Connect failed! retry after 5s");
        sleep(5);
    } while (1);

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

int32_t AliIot::iot_post_event_warn(uint32_t devid, const char *value)
{
    int32_t res = STATE_USER_INPUT_BASE;
    char *event_id = "warn";
    char *event_payload = NULL;
    uint32_t event_payload_len = 0;

    if (value == NULL)
    {
        return STATE_USER_INPUT_NULL_POINTER;
    }
    
    event_payload_len = strlen("warn_code") + strlen(value) + 10;
    event_payload = HAL_Malloc(event_payload_len);
    if (event_payload == NULL)
    {
        return STATE_SYS_DEPEND_MALLOC;
    }
    memset(event_payload, 0, event_payload_len);

    HAL_Snprintf(event_payload, event_payload_len, "{\"warn_code\": %s}", value);


    res = IOT_Linkkit_TriggerEvent(devid, event_id, strlen(event_id),
                                   event_payload, strlen(event_payload));
    HAL_Free(event_payload);

    return res;
}


int32_t AliIot::iot_post_event_error(uint32_t devid, const char *value)
{
    int32_t res = STATE_USER_INPUT_BASE;
    char *event_id = "error";
    char *event_payload = NULL;
    uint32_t event_payload_len = 0;

    if (value == NULL)
    {
        return STATE_USER_INPUT_NULL_POINTER;
    }

    event_payload_len = strlen("error_code") + strlen(value) + 10;
    event_payload = HAL_Malloc(event_payload_len);
    if (event_payload == NULL)
    {
        return STATE_SYS_DEPEND_MALLOC;
    }
    memset(event_payload, 0, event_payload_len);

    HAL_Snprintf(event_payload, event_payload_len, "{\"error_code\": %s}", value);

    res = IOT_Linkkit_TriggerEvent(devid, event_id, strlen(event_id),
                                   event_payload, strlen(event_payload));
    HAL_Free(event_payload);
    return res;
}

int32_t AliIot::iot_post_property_status(uint32_t devid, uint32_t value)
{
    int32_t res = STATE_USER_INPUT_BASE;
    char property_payload[64] = {0};

    res = HAL_Snprintf(property_payload, sizeof(property_payload), "{\"status\": %d}", value);
    if (res < 0) {
        return STATE_SYS_DEPEND_SNPRINTF;
    }

    res = IOT_Linkkit_Report(devid, ITM_MSG_POST_PROPERTY,
                            (uint8_t *)property_payload, strlen(property_payload));
    return res;
}

int32_t AliIot::iot_parse_property(const char *request, int request_len)
{
    cJSON *req = cJSON_Parse(request);
    if (req == NULL || !cJSON_IsObject(req))
    {
        return STATE_DEV_MODEL_WRONG_JSON_FORMAT;
    }

    cJSON_Delete(req);
    return 0;
}

int AliIot::iot_connected_event_handler(void)
{
    ROS_INFO("Cloud Connected");
    g_user_iot_ctx.cloud_connected = 1;

    return 0;
}

int AliIot::iot_disconnected_event_handler(void)
{
    ROS_INFO("Cloud Disconnected");
    g_user_iot_ctx.cloud_connected = 0;

    return 0;
}

int AliIot::iot_initialized(const int devid)
{
    ROS_INFO("Device Initialized");
    g_user_iot_ctx.master_initialized = 1;
    return 0;
}

int AliIot::iot_report_reply_event_handler(const int devid, const int msgid, const int code, const char *reply, const int reply_len)
{
    ROS_INFO("Message Post Reply Received, Message ID: %d, Code: %d, Reply: %.*s", msgid, code, reply_len, (reply == NULL) ? ("NULL") : (reply));
    return 0;
}

int AliIot::iot_trigger_event_reply_event_handler(const int devid, const int msgid, const int code, const char *eventid,
                                                  const int eventid_len, const char *message, const int message_len)
{
    ROS_INFO("Trigger Event Reply Received, Message ID: %d, Code: %d, EventID: %.*s, Message: %.*s",
             msgid, code,
             eventid_len,
             eventid, message_len, message);
    return 0;
}

int AliIot::iot_property_set_event_handler(const int devid, const char *request, const int request_len)
{
    int res = 0;
    ROS_INFO("Property Set Received, Request: %s", request);

    res = IOT_Linkkit_Report(IOT_MASTER_DEVID, ITM_MSG_POST_PROPERTY,
                             (unsigned char *)request, request_len);
    ROS_INFO("Post Property return: %d", res);

    return 0;
}

int AliIot::iot_service_request_event_handler(const int devid, const char *serviceid, const int serviceid_len, const char *request,
                                              const int request_len, char **response, int *response_len)
{
    char *result = "\"success\"";
    cJSON *root = NULL, *command = NULL;
    const char *response_fmt = "{\"ret\": %s}";

    ROS_INFO("Service Request Received, Service ID: %.*s, Payload: %s", serviceid_len, serviceid, request);

    root = cJSON_Parse(request);
    if (root == NULL ||!cJSON_IsObject(root))
    {
        ROS_INFO("JSON Parse Error");
        return -1;
    }

    if (strlen("control") == serviceid_len && memcmp("control", serviceid, serviceid_len) == 0)
    {
        command = cJSON_GetObjectItem(root, "cmd");
        if (command == NULL || !cJSON_IsString(command))
        {
            cJSON_Delete(root);
            return -1;
        }

        cmd = command->valuestring;

        *response_len = strlen(response_fmt) + 10 + strlen(result);
        *response = (char *)HAL_Malloc(*response_len);
        if (*response == NULL)
        {
            ROS_INFO("Memory Not Enough");
            return -1;
        }
        memset(*response, 0, *response_len);
        HAL_Snprintf(*response, *response_len, response_fmt, result);
        *response_len = strlen(*response);
    }

    cJSON_Delete(root);
    return 0;
}

int AliIot::iot_timestamp_reply_event_handler(const char *timestamp)
{
    ROS_INFO("Current Timestamp: %s", timestamp);
    return 0;
}

int AliIot::iot_fota_event_handler(int type, const char *version)
{
    char buffer[128] = {0};
    int buffer_length = 128;

    if (type == 0)
    {
        ROS_INFO("New Firmware Version: %s", version);
        IOT_Linkkit_Query(IOT_MASTER_DEVID, ITM_MSG_QUERY_FOTA_DATA, (unsigned char *)buffer, buffer_length);
    }

    return 0;
}

int AliIot::iot_cloud_error_handler(const int code, const char *data, const char *detail)
{
    ROS_INFO("code =%d ,data=%s, detail=%s", code, data, detail);
    return 0;
}

int AliIot::iot_dynreg_device_secret(const char *device_secret)
{
    ROS_INFO("device secret: %s", device_secret);
    return 0;
}

int AliIot::iot_sdk_state_dump(int ev, const char *msg)
{
    ROS_INFO("received state event, -0x%04x(%s)\n", -ev, msg);
    return 0;
}

void AliIot::intervalTopicCallback(const ros::TimerEvent &)
{
    iot_publish(pclient);
}

void AliIot::intervalLinkkitCallback(const ros::TimerEvent &)
{
    uint32_t state = 1;
    if (strcmp(demo_status.status.c_str(), "running") == 0)
        state = 0;
    uint32_t ret = iot_post_property_status(g_user_iot_ctx.master_devid, state);
    //uint32_t ret = iot_post_event_warn(g_user_iot_ctx.master_devid, "\"test\"");
    //uint32_t ret = iot_post_event_error(g_user_iot_ctx.master_devid, "\"test\"");
}

void AliIot::intervalCallback(const ros::TimerEvent &)
{
    IOT_MQTT_Yield(pclient, 200);
    IOT_Linkkit_Yield(200);

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
