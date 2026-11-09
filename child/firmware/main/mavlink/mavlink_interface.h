#ifndef MAVLINK_INTERFACE_H
#define MAVLINK_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool heartbeat_received;
    bool attitude_received;
    bool position_received;
    bool battery_received;

    uint8_t system_status;
    uint8_t base_mode;
    uint32_t custom_mode;

    float roll;
    float pitch;
    float yaw;

    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;

    float battery_voltage;
    int8_t battery_remaining;
} px4_telemetry_t;

void mavlink_interface_init(void);
void mavlink_interface_parse_byte(uint8_t byte);
void mavlink_interface_get_telemetry(px4_telemetry_t *telemetry);

#endif