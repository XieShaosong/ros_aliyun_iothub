#include "ros/ros.h"
#include "ros_aliyun_iothub/status.h"
#include "ros_aliyun_iothub/control.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <pwd.h>

#include "infra_config.h"
#include "infra_types.h"
#include "infra_defs.h"
#include "infra_compat.h"
#include "infra_state.h"
#include "dev_model_api.h"
#include "dev_sign_api.h"
#include "mqtt_api.h"
#include "http2_upload_api.h"
#include "wrappers.h"
#include "cJSON.h"

#define IOT_MASTER_DEVID (0)
#define IOT_YIELD_TIMEOUT_MS (200)

#define HTTP2_ONLINE_SERVER_URL "a1SRxGfxtwd.iot-as-http2.cn-shanghai.aliyuncs.com"
#define HTTP2_ONLINE_SERVER_PORT 443

using namespace std;

namespace ros_iot
{
class AliIot
{
public:
    AliIot();

    ros::NodeHandle n;
    ros::Timer topic_timer, linkkit_timer, timer;
    ros::Subscriber status_sub;
    ros::ServiceClient ctrl_client;
    static string cmd;
    void intervalTopicCallback(const ros::TimerEvent &);
    void intervalLinkkitCallback(const ros::TimerEvent &);
    void intervalCallback(const ros::TimerEvent &);
    ros_aliyun_iothub::status demo_status;
    void statusCallback(const ros_aliyun_iothub::status::ConstPtr &msg);
    void handleCommand(const char *cmd);
public:
    char product_key[IOTX_PRODUCT_KEY_LEN + 1] = "a1SRxGfxtwd";
    char product_secret[IOTX_PRODUCT_SECRET_LEN + 1] = "";
    char device_name[IOTX_DEVICE_NAME_LEN + 1] = "robot_1";
    char device_secret[IOTX_DEVICE_SECRET_LEN + 1] = "din3Q16zbzPUwASlUrDD3dVYpk1gDMOJ";

    typedef struct
    {
        int master_devid;
        int cloud_connected;
        int master_initialized;
    } iot_ctx_t;
    static iot_ctx_t g_user_iot_ctx;

    void *pclient = NULL;
    iotx_mqtt_param_t mqtt_params;
    void iot_init();
    bool iot_publish(void *handle);
    bool iot_subscribe(void *handle);
    static void iot_message_arrive(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg);
    static void iot_event_handle(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg);
    static int iot_everything_state_handle(const int state_code, const char *state_message);
    static int iot_identity_response_handle(const char *payload);
    int32_t iot_post_event_warn(uint32_t devid, const char *value);
    int32_t iot_post_event_error(uint32_t devid, const char *value);
    int32_t iot_post_property_status(uint32_t devid, uint32_t value);
    int32_t iot_parse_property(const char *request, int request_len);
    static int iot_connected_event_handler(void);
    static int iot_disconnected_event_handler(void);
    static int iot_initialized(const int devid);
    static int iot_report_reply_event_handler(const int devid, const int msgid, const int code, const char *reply, const int reply_len);
    static int iot_trigger_event_reply_event_handler(const int devid, const int msgid, const int code, const char *eventid,
                                                     const int eventid_len, const char *message, const int message_len);
    static int iot_property_set_event_handler(const int devid, const char *request, const int request_len);
    static int iot_service_request_event_handler(const int devid, const char *serviceid, const int serviceid_len, const char *request,
                                                 const int request_len, char **response, int *response_len);
    static int iot_timestamp_reply_event_handler(const char *timestamp);
    static int iot_fota_event_handler(int type, const char *version);
    static int iot_cloud_error_handler(const int code, const char *data, const char *detail);
    static int iot_dynreg_device_secret(const char *device_secret);
    static int iot_sdk_state_dump(int ev, const char *msg);
    
    static int upload_end;
    static char g_upload_id[50];
    void iot_upload_file_result(const char *file_path, int result, void *user_data);
    void iot_upload_id_received_handle(const char *file_path, const char *upload_id, void *user_data);
    int iot_uploadfile();
};
}
