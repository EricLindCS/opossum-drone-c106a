#include "mavlink_interface.h"

#include <string.h>
#include "esp_log.h"
#include "common/mavlink.h"

static const char *TAG = "MAVLINK";

static mavlink_message_t mavlink_msg;
static mavlink_status_t mavlink_status;
static px4_telemetry_t telemetry;

void mavlink_interface_init(void)
{
    memset(&mavlink_msg, 0, sizeof(mavlink_msg));
    memset(&mavlink_status, 0, sizeof(mavlink_status));
    memset(&telemetry, 0, sizeof(telemetry));

    telemetry.battery_remaining = -1;

    ESP_LOGI(TAG, "MAVLink parser initialized");
}

void mavlink_interface_parse_byte(uint8_t byte)
{
    uint8_t result = mavlink_parse_char(
        MAVLINK_COMM_0,
        byte,
        &mavlink_msg,
        &mavlink_status
    );

    if (result != MAVLINK_FRAMING_OK) {
        return;
    }

    switch (mavlink_msg.msgid) {

        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&mavlink_msg, &heartbeat);

            telemetry.heartbeat_received = true;
            telemetry.system_status = heartbeat.system_status;
            telemetry.base_mode = heartbeat.base_mode;
            telemetry.custom_mode = heartbeat.custom_mode;

            ESP_LOGI(TAG,
                "HEARTBEAT: system=%u component=%u mode=%lu status=%u",
                mavlink_msg.sysid,
                mavlink_msg.compid,
                (unsigned long)heartbeat.custom_mode,
                heartbeat.system_status
            );
            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&mavlink_msg, &attitude);

            telemetry.attitude_received = true;
            telemetry.roll = attitude.roll;
            telemetry.pitch = attitude.pitch;
            telemetry.yaw = attitude.yaw;
            break;
        }

        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
            mavlink_local_position_ned_t position;
            mavlink_msg_local_position_ned_decode(
                &mavlink_msg,
                &position
            );

            telemetry.position_received = true;
            telemetry.x = position.x;
            telemetry.y = position.y;
            telemetry.z = position.z;
            telemetry.vx = position.vx;
            telemetry.vy = position.vy;
            telemetry.vz = position.vz;
            break;
        }

        case MAVLINK_MSG_ID_BATTERY_STATUS: {
            mavlink_battery_status_t battery;
            mavlink_msg_battery_status_decode(&mavlink_msg, &battery);

            telemetry.battery_received = true;

            if (battery.voltages[0] != UINT16_MAX) {
                telemetry.battery_voltage =
                    battery.voltages[0] / 1000.0f;
            }

            telemetry.battery_remaining =
                battery.battery_remaining;
            break;
        }

        default:
            break;
    }
}

void mavlink_interface_get_telemetry(px4_telemetry_t *output)
{
    if (output != NULL) {
        *output = telemetry;
    }
}