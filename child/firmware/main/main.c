/* main.c — ocelotbot drone ESP32-C6 Firmware
 * adapted from esp32-micro-ros-px4-offboard example
 *
 * ROS topics (subscribed):
 *   /gcs/drone_{ID}/command    std_msgs/String  — flight commands
 *   /gcs/drone_{ID}/config     std_msgs/String  — feature config
 *   /gcs/drone_{ID}/control    PoseStamped      — position setpoints (m, positive-up Z)
 *
 * ROS topics (published):
 *   /drone_{ID}/state              std_msgs/String         — state machine
 *   /drone_{ID}/role               std_msgs/String         — role (idle)
 *   /drone_{ID}/battery            std_msgs/Int8           — battery % (0–100, -1=unknown)
 *
 * MAVLink → PX4 (UART1 57600):
 *   Heartbeat,
 *   COMPONENT_ARM_DISARM, SET_MODE,
 *   SET_POSITION_TARGET_LOCAL_NED,
 *   NAV_LAND
 *
 * Takeoff altitude options:
 *   A: 1.0 m  — tight formations only, >1.2 m H spacing required
 *   B: 1.5 m  — recommended, clears ground effect, absorbs baro drift  ← default
 *   C: 2.0 m  — close proximity swarm (<0.5 m H), best downwash margin
 *
 */

#ifndef __STDC_WANT_LIB_EXT1__
#define __STDC_WANT_LIB_EXT1__ 0
#endif

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <sys/types.h>  /* Picolibc: declare system types before lwIP headers */
#include <math.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_err.h"
#include "nvs_flash.h"


#include <common/mavlink.h>
#include "driver/uart.h"
#include "driver/gpio.h"

#include <rosidl_runtime_c/string_functions.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "esp_netif.h"

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int8.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <uros_network_interfaces.h>


static const char *TAG = "drone";

/* ── Identity ──────────────────────────────────────────────────────────── */
#define DRONE_ID          1
#define DRONE_ID_LED_PIN  GPIO_NUM_15

/* ── Network ───────────────────────────────────────────────────────────── */
/* Wi-Fi SSID/password and micro-ROS agent IP/port are configured in
 * idf.py menuconfig. The ESP32 uses DHCP from the phone hotspot.
 * Do not assign a static IP: phone hotspot subnets vary by device. */

/* ── MAVLink / PX4 ─────────────────────────────────────────────────────── */
#define UART_TX     GPIO_NUM_16  /* XIAO ESP32-C6 D6 / TX */
#define UART_RX     GPIO_NUM_17  /* XIAO ESP32-C6 D7 / RX */
#define GCS_SYSID   42
#define GCS_COMPID  200
#define PX4_SYSID   1
#define PX4_COMPID  1

/* PX4 custom flight modes (main_mode << 16) */
#define PX4_MODE_STABILIZED  0x00070000UL   /* RC takes control */
#define PX4_MODE_OFFBOARD    0x00060000UL   /* onboard computer control */

/* ── C2 watchdog ───────────────────────────────────────────────────────── */
/* C2_PING_TIMEOUT_MS must be kept short: rmw_uros_ping_agent() is a
 * blocking call that stalls the executor for up to TIMEOUT × ATTEMPTS.
 * was enough to drop PX4 EKF2's external-position lock when several drones
 * connect simultaneously and the agent is under load — causing ARM rejection.
 * 100 ms × 2 = 200 ms max block; well within EKF2 tolerance on local WiFi. */
#define C2_PING_TIMEOUT_MS   100   /* per ping attempt */
#define C2_PING_ATTEMPTS     2     /* attempts per check */
#define C2_CHECK_INTERVAL_MS 1000  /* interval between checks */
#define C2_FAIL_THRESHOLD    2     /* consecutive failures before ELAND */

/* ── Mission parameters ────────────────────────────────────────────────── */
#define MISSION_TAKEOFF_ALT_M      1.5f            /* Option B — change to 2.0 for tight swarm */
#define MISSION_TAKEOFF_WAIT_MS    5000
#define MISSION_MAX_HOVER_MS       (15UL * 60UL * 1000UL)  /* 15 min safety timeout */
#define MISSION_LAND_DESCEND_MS    5000
#define OFFBOARD_STREAM_PERIOD_MS  50              /* 20 Hz setpoint stream */

#define MICRO_ROS_AGENT_IP "172.20.10.4"
#define MICRO_ROS_AGENT_PORT 8888

/* ── micro-ROS macros ──────────────────────────────────────────────────── */
#define RCCHECK(fn) \
    { rcl_ret_t _rc = (fn); \
      if (_rc != RCL_RET_OK) { \
          printf("Failed line %d: %d. Aborting.\n", __LINE__, (int)_rc); \
          vTaskDelete(NULL); } }
#define RCSOFTCHECK(fn) \
    { rcl_ret_t _rc = (fn); \
      if (_rc != RCL_RET_OK) \
          printf("Soft fail line %d: %d.\n", __LINE__, (int)_rc); }

/* ══════════════════════════════════════════════════════════════════════════
 * State machine
 * ══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    DRONE_DISARMED = 0,
    DRONE_ARMED,
    DRONE_MISSION,
    DRONE_RETURNING_HOME,
    DRONE_LANDING,
    DRONE_KILLED,
} drone_state_t;

static volatile drone_state_t drone_state = DRONE_DISARMED;
static volatile bool          state_dirty = true;

static const char * const state_names[] = {
    "disarmed", "armed", "mission", "returning_home", "landing", "killed"
};

/* Home position in PX4 local NED coordinates, captured on ARM. */
static volatile float home_x = 0.0f, home_y = 0.0f, home_z = 0.0f;

/* Battery status from PX4 BATTERY_STATUS MAVLink message (0–100, -1 = unknown) */
static volatile int8_t battery_remaining_pct = -1;

/* PX4 local position (LOCAL_POSITION_NED, z negated to up-positive).
 * Used to capture the local origin when COMMAND_ARM is received. */
static volatile float px4_pos_x = 0.0f, px4_pos_y = 0.0f, px4_pos_z = 0.0f;
static volatile bool  px4_pos_valid = false;

/* Mission setpoint from /gcs/drone_{ID}/control (positive-up Z, yaw in radians) */
static volatile float setpoint_x = 0.0f, setpoint_y = 0.0f, setpoint_z = 1.5f;
static volatile float setpoint_yaw = 0.0f;   /* radians, 0 = North, CW positive */
static volatile bool  setpoint_received = false;

/* Control source selected by the GCS. */
static volatile bool gcs_control_active = false;


/* FreeRTOS task handles */
static TaskHandle_t mission_task_handle     = NULL;
static TaskHandle_t return_home_task_handle = NULL;
static TaskHandle_t eland_task_handle       = NULL;
static TaskHandle_t prearm_stream_handle    = NULL;

/* ══════════════════════════════════════════════════════════════════════════
 * UART / MAVLink
 * ══════════════════════════════════════════════════════════════════════════ */

static void uart_mavlink_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART_TX, UART_RX,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void mav_send(const mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    uart_write_bytes(UART_NUM_1, (const char *)buf, len);
}

/* Heartbeat — identifies this device as onboard controller to PX4 */
static void send_heartbeat_once(void)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(GCS_SYSID, GCS_COMPID, &msg,
        MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
        0, 0, MAV_STATE_ACTIVE);
    mav_send(&msg);
}

/* ── MAVLink flight helpers ────────────────────────────────────────────── */

static void mav_arm(bool arm)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(GCS_SYSID, GCS_COMPID, &msg,
        PX4_SYSID, PX4_COMPID,
        MAV_CMD_COMPONENT_ARM_DISARM, 0,
        arm ? 1.0f : 0.0f, 0, 0, 0, 0, 0, 0);
    mav_send(&msg);
    ESP_LOGI(TAG, "MAV: %s", arm ? "ARM" : "DISARM");
}

/* Force-disarm regardless of flight state (KILL) */
static void mav_kill(void)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(GCS_SYSID, GCS_COMPID, &msg,
        PX4_SYSID, PX4_COMPID,
        MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0.0f, 21196.0f, 0, 0, 0, 0, 0);
    mav_send(&msg);
    ESP_LOGI(TAG, "MAV: KILL (force disarm)");
}

/* Emergency land — NAV_LAND at current horizontal position */
static void mav_eland(void)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(GCS_SYSID, GCS_COMPID, &msg,
        PX4_SYSID, PX4_COMPID,
        MAV_CMD_NAV_LAND, 0,
        0, 0, 0, 0, NAN, NAN, NAN);
    mav_send(&msg);
    ESP_LOGI(TAG, "MAV: EMERGENCY LAND");
}

/* SET_POSITION_TARGET_LOCAL_NED — position-only, NED frame.
 * z_ned: negative = upward (e.g. -1.5 for 1.5 m altitude). */
static void mav_set_position_ned(float x, float y, float z_ned)
{
    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(
        GCS_SYSID, GCS_COMPID, &msg,
        (uint32_t)(esp_timer_get_time() / 1000),
        PX4_SYSID, PX4_COMPID,
        MAV_FRAME_LOCAL_NED,
        0b0000111111111000,   /* type_mask: position only, yaw ignored */
        x, y, z_ned,
        0, 0, 0,
        0, 0, 0,
        0, 0);
    mav_send(&msg);
}

/* SET_POSITION_TARGET_LOCAL_NED — position + yaw, NED frame.
 * yaw: radians, 0 = North, positive = clockwise (NED convention). */
static void mav_set_position_yaw_ned(float x, float y, float z_ned, float yaw)
{
    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(
        GCS_SYSID, GCS_COMPID, &msg,
        (uint32_t)(esp_timer_get_time() / 1000),
        PX4_SYSID, PX4_COMPID,
        MAV_FRAME_LOCAL_NED,
        0b0000101111111000,   /* type_mask: position + yaw, yaw_rate ignored */
        x, y, z_ned,
        0, 0, 0,
        0, 0, 0,
        yaw, 0);
    mav_send(&msg);
}

/* Set PX4 flight mode via SET_MODE */
static void mav_set_mode(uint32_t custom_mode)
{
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(GCS_SYSID, GCS_COMPID, &msg,
        PX4_SYSID,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode);
    mav_send(&msg);
    ESP_LOGI(TAG, "MAV: SET_MODE 0x%08lX", (unsigned long)custom_mode);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Drone ID LED burst
 * ══════════════════════════════════════════════════════════════════════════ */

static void drone_id_led_update(void)
{
    static TickType_t last_tick = 0;
    static int state = 0, blink_index = 0;
    const TickType_t on_time     = pdMS_TO_TICKS(120);
    const TickType_t off_time    = pdMS_TO_TICKS(180);
    const TickType_t burst_pause = pdMS_TO_TICKS(1800);
    TickType_t now = xTaskGetTickCount();
    switch (state) {
        case 0:
            if ((now - last_tick) >= burst_pause) {
                blink_index = 0; gpio_set_level(DRONE_ID_LED_PIN, 1);
                last_tick = now; state = 1; }
            break;
        case 1:
            if ((now - last_tick) >= on_time) {
                gpio_set_level(DRONE_ID_LED_PIN, 0);
                last_tick = now; blink_index++; state = 2; }
            break;
        case 2:
            if (blink_index >= DRONE_ID) { state = 0; last_tick = now; }
            else if ((now - last_tick) >= off_time) {
                gpio_set_level(DRONE_ID_LED_PIN, 1); last_tick = now; state = 1; }
            break;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * micro-ROS resources
 * ══════════════════════════════════════════════════════════════════════════ */

static rcl_publisher_t    publisher_state;
static rcl_publisher_t    publisher_role;
static rcl_publisher_t    publisher_battery;

static rcl_subscription_t command_sub;
static rcl_subscription_t config_sub;
static rcl_subscription_t control_sub;

static std_msgs__msg__String             command_msg;
static std_msgs__msg__String             config_msg;
static geometry_msgs__msg__PoseStamped   control_msg;
static std_msgs__msg__String             state_pub_msg;
static std_msgs__msg__String             role_pub_msg;
static std_msgs__msg__Int8               battery_pub_msg;


static char topic_gcs_command[64];
static char topic_gcs_config[64];
static char topic_gcs_control[64];
static char topic_state[64];
static char topic_role[64];
static char topic_battery[64];
static char drone_ns[16];

/* ══════════════════════════════════════════════════════════════════════════
 * Helpers
 * ══════════════════════════════════════════════════════════════════════════ */

static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

/* ══════════════════════════════════════════════════════════════════════════
 * Pre-arm setpoint stream task
 *
 * Streams SET_POSITION_TARGET_LOCAL_NED at 20 Hz from the moment COMMAND_ARM
 * is received until the drone transitions out of DRONE_ARMED (→ MISSION or
 * DISARM).  This gives PX4 EKF2 the maximum possible convergence time between
 * ARM and MISSION_START, regardless of how quickly the operator presses the
 * buttons.  mission_task_fn Phase 1 then overlaps for its own 3 s window.
 * ══════════════════════════════════════════════════════════════════════════ */
static void prearm_stream_task_fn(void *arg)
{
    ESP_LOGI("mission", "Prearm stream started");
    while (drone_state == DRONE_ARMED) {
        mav_set_position_ned(home_x, home_y, -0.1f);
        vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
    }
    ESP_LOGI("mission", "Prearm stream stopped (state→%d)", (int)drone_state);
    prearm_stream_handle = NULL;
    vTaskDelete(NULL);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Mission FreeRTOS task
 *
 * Sequence:
 *   1. Stream idle setpoints 1 s  → PX4 accepts OFFBOARD switch (prearm stream
 *      has been running since ARM, so EKF2 convergence time = ARM→MISSION + 1 s)
 *   2. OFFBOARD → ARM
 *   3. Climb to MISSION_TAKEOFF_ALT_M (5 s)
 *   4. Hover and follow /gcs/drone_X/control setpoints for up to 15 min.
 *      Re-reads setpoint_x/y/z every 50 ms — new waypoints take effect within
 *      one tick.  Hovers at home if no setpoint received yet.
 *   5. Auto-land after 15 min timeout (or on ELAND/KILL command)
 * ══════════════════════════════════════════════════════════════════════════ */

static void mission_task_fn(void *arg)
{
    TickType_t t0;
    uint32_t elapsed;

    /* Phase 1: pre-arm stream (PX4 requires >2 Hz setpoints before OFFBOARD) */
    t0 = xTaskGetTickCount();
    do {
        if (drone_state != DRONE_MISSION) goto mission_abort;
        mav_set_position_ned(home_x, home_y, -0.1f);
        vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
        elapsed = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
    } while (elapsed < 1000);

    /* Phase 2: OFFBOARD → arm.
     * PX4 requires OFFBOARD mode to ARM without GPS — the GPS pre-arm check is
     * waived once the external position setpoints are flowing.  Setpoints must
     * keep streaming during the switch; a bare vTaskDelay would let PX4 time
     * out of OFFBOARD and fall back to its default mode where ARM is rejected. */
    mav_set_mode(PX4_MODE_OFFBOARD);
    {
        TickType_t t_ob = xTaskGetTickCount();
        while ((uint32_t)((xTaskGetTickCount() - t_ob) * portTICK_PERIOD_MS) < 500) {
            if (drone_state != DRONE_MISSION) goto mission_abort;
            mav_set_position_ned(home_x, home_y, -0.1f);
            vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
        }
    }
    mav_arm(true);

    /* Phase 3: climb */
    t0 = xTaskGetTickCount();
    do {
        if (drone_state != DRONE_MISSION) goto mission_abort;
        mav_set_position_ned(home_x, home_y, -MISSION_TAKEOFF_ALT_M);
        vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
        elapsed = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
    } while (elapsed < MISSION_TAKEOFF_WAIT_MS);

    /* Phase 4: hover and follow setpoints for up to MISSION_MAX_HOVER_MS.
     * setpoint_x/y/z are re-read every tick so a new waypoint published to
     * /gcs/drone_X/control takes effect within 50 ms.
     * Falls back to (home_x, home_y, cruise_alt) if no setpoint received yet. */
    t0 = xTaskGetTickCount();
    do {
        if (drone_state != DRONE_MISSION) goto mission_abort;
        float sp_x   = setpoint_received ? setpoint_x   : home_x;
        float sp_y   = setpoint_received ? setpoint_y   : home_y;
        float sp_z   = setpoint_received ? -setpoint_z  : -MISSION_TAKEOFF_ALT_M;
        float sp_yaw = setpoint_received ? setpoint_yaw : 0.0f;
        mav_set_position_yaw_ned(sp_x, sp_y, sp_z, sp_yaw);
        vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
        elapsed = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
    } while (elapsed < MISSION_MAX_HOVER_MS);
    ESP_LOGW(TAG, "Mission 15 min timeout — auto-landing");

    /* Phase 5: land on spot */
    drone_state = DRONE_LANDING;
    state_dirty = true;

    mav_eland();
    vTaskDelay(pdMS_TO_TICKS(MISSION_LAND_DESCEND_MS));

    mav_arm(false);
    drone_state = DRONE_DISARMED;
    state_dirty = true;
    goto mission_done;

mission_abort:
    ESP_LOGW(TAG, "Mission aborted (state override)");
mission_done:
    mission_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Return-home FreeRTOS task
 * ══════════════════════════════════════════════════════════════════════════ */

static void return_home_task_fn(void *arg)
{
    TickType_t t0 = xTaskGetTickCount();
    uint32_t elapsed;

    /* Stream home position above home XY at cruise altitude */
    do {
        if (drone_state != DRONE_RETURNING_HOME) goto rh_done;
        mav_set_position_ned(home_x, home_y, -MISSION_TAKEOFF_ALT_M);
        vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
        elapsed = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
    } while (elapsed < 5000);

    /* Land */
    drone_state = DRONE_LANDING;
    state_dirty = true;
    mav_eland();
    vTaskDelay(pdMS_TO_TICKS(MISSION_LAND_DESCEND_MS));

    mav_arm(false);
    drone_state = DRONE_DISARMED;
    state_dirty = true;

rh_done:
    return_home_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Emergency-land FreeRTOS task
 * ══════════════════════════════════════════════════════════════════════════ */

static void eland_task_fn(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(MISSION_LAND_DESCEND_MS));
    mav_arm(false);
    drone_state = DRONE_DISARMED;
    state_dirty = true;
    eland_task_handle = NULL;
    vTaskDelete(NULL);
}

static void trigger_eland(void)
{
    if (mission_task_handle)     { vTaskDelete(mission_task_handle);     mission_task_handle = NULL; }
    if (return_home_task_handle) { vTaskDelete(return_home_task_handle); return_home_task_handle = NULL; }
    if (eland_task_handle)       { vTaskDelete(eland_task_handle);       eland_task_handle = NULL; }
    mav_eland();
    drone_state = DRONE_LANDING;
    state_dirty = true;
    xTaskCreate(eland_task_fn, "eland", 2048, NULL, 5, &eland_task_handle);
}


/* Publish state immediately — called after any state transition inside the
 * executor so the GCS receives the new state without waiting up to 100 ms
 * for the next timer_callback tick.  Safe to call from executor callbacks
 * (publisher is initialised before the executor loop starts). */
static void publish_state_now(void)
{
    state_dirty = false;
    const char *s = state_names[(int)drone_state];
    rosidl_runtime_c__String__assign(&state_pub_msg.data, s);
    RCSOFTCHECK(rcl_publish(&publisher_state, &state_pub_msg, NULL));
    ESP_LOGI(TAG, "State → %s (immediate)", s);
}

/* ══════════════════════════════════════════════════════════════════════════
 * GCS command callback  (/gcs/drone_{ID}/command)
 * ══════════════════════════════════════════════════════════════════════════ */

static void command_callback(const void *msg_in)
{
    const std_msgs__msg__String *m = (const std_msgs__msg__String *)msg_in;
    if (!m || !m->data.data || m->data.size == 0) return;

    char buf[64] = {0};
    size_t n = m->data.size < 63 ? m->data.size : 63;
    memcpy(buf, m->data.data, n);
    for (int i = (int)n - 1; i >= 0; i--) {
        if (buf[i]==' '||buf[i]=='\n'||buf[i]=='\r'||buf[i]=='\t') buf[i]='\0'; else break;
    }
    ESP_LOGI(TAG, "CMD: %s", buf);

    if (strcmp(buf, "COMMAND_ARM") == 0) {
        /* Capture the PX4 local-position origin for subsequent setpoints. */
        if (px4_pos_valid) {
            home_x = px4_pos_x;
            home_y = px4_pos_y;
            home_z = px4_pos_z;
            ESP_LOGI(TAG, "PX4 home captured: (%.2f, %.2f, %.2f)", home_x, home_y, home_z);
        }
        /* Do NOT call mav_arm here — PX4 is not in OFFBOARD yet and will reject it.
         * The real arm happens in mission_task_fn Phase 2, after the prestream converges EKF2. */
        drone_state = DRONE_ARMED;
        publish_state_now();   /* immediate — GCS needs "armed" before sending MISSION_START */
        /* Start streaming setpoints immediately so EKF2 converges during ARM→MISSION window */
        if (!prearm_stream_handle)
            xTaskCreate(prearm_stream_task_fn, "prearm", 2048, NULL, 4, &prearm_stream_handle);

    } else if (strcmp(buf, "COMMAND_DISARM") == 0) {
        if (prearm_stream_handle) {
            vTaskDelete(prearm_stream_handle);
            prearm_stream_handle = NULL;
        }
        mav_arm(false);
        drone_state = DRONE_DISARMED;
        publish_state_now();

    } else if (strcmp(buf, "COMMAND_MISSION_START") == 0) {
        if (drone_state != DRONE_ARMED && drone_state != DRONE_RETURNING_HOME) {
            ESP_LOGW(TAG, "Mission blocked — state: %s", state_names[drone_state]);
            return;
        }
        if (return_home_task_handle) {
            vTaskDelete(return_home_task_handle);
            return_home_task_handle = NULL;
        }
        setpoint_received = false;   /* clear stale setpoint/yaw from previous mission */
        setpoint_yaw = 0.0f;
        drone_state = DRONE_MISSION;
        publish_state_now();
        xTaskCreate(mission_task_fn, "mission", 4096, NULL, 5, &mission_task_handle);

    } else if (strcmp(buf, "COMMAND_RETURN_HOME") == 0) {
        if (drone_state != DRONE_MISSION && drone_state != DRONE_RETURNING_HOME) {
            ESP_LOGW(TAG, "RTH blocked — not in flight");
            return;
        }
        if (mission_task_handle) {
            vTaskDelete(mission_task_handle);
            mission_task_handle = NULL;
        }
        drone_state = DRONE_RETURNING_HOME;
        publish_state_now();
        mav_set_mode(PX4_MODE_OFFBOARD);
        xTaskCreate(return_home_task_fn, "return_home", 4096, NULL, 5, &return_home_task_handle);

    } else if (strcmp(buf, "COMMAND_ELAND") == 0) {
        trigger_eland();

    } else if (strcmp(buf, "COMMAND_KILL") == 0) {
        if (mission_task_handle)     { vTaskDelete(mission_task_handle);     mission_task_handle = NULL; }
        if (return_home_task_handle) { vTaskDelete(return_home_task_handle); return_home_task_handle = NULL; }
        mav_kill();
        drone_state = DRONE_KILLED;
        publish_state_now();

    } else {
        ESP_LOGW(TAG, "CMD: unknown '%s'", buf);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * GCS config callback  (/gcs/drone_{ID}/config)
 * ══════════════════════════════════════════════════════════════════════════ */

static void config_callback(const void *msg_in)
{
    const std_msgs__msg__String *m = (const std_msgs__msg__String *)msg_in;
    if (!m || !m->data.data || m->data.size == 0) return;

    char buf[64] = {0};
    size_t n = m->data.size < 63 ? m->data.size : 63;
    memcpy(buf, m->data.data, n);
    for (int i = (int)n - 1; i >= 0; i--) {
        if (buf[i]==' '||buf[i]=='\n'||buf[i]=='\r'||buf[i]=='\t') buf[i]='\0'; else break;
    }
    ESP_LOGI(TAG, "CFG: %s", buf);

    if (strcmp(buf, "CONFIG_SOURCE_RC") == 0) {
        gcs_control_active = false;
        mav_set_mode(PX4_MODE_STABILIZED);   /* RC takes over */

    } else if (strcmp(buf, "CONFIG_SOURCE_GCS") == 0) {
        gcs_control_active = true;
        mav_set_mode(PX4_MODE_OFFBOARD);     /* onboard controller */

    } else {
        ESP_LOGW(TAG, "CFG: unknown '%s'", buf);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * GCS control callback  (/gcs/drone_{ID}/control)
 *
 * Receives position setpoints as PoseStamped:
 *   position.x/y  — horizontal NED metres
 *   position.z    — altitude in metres, positive = up
 *                   converted to NED z_ned = -position.z before sending
 * ══════════════════════════════════════════════════════════════════════════ */

static void control_callback(const void *msg_in)
{
    const geometry_msgs__msg__PoseStamped *msg =
        (const geometry_msgs__msg__PoseStamped *)msg_in;
    if (!msg) return;

    setpoint_x = (float)msg->pose.position.x;
    setpoint_y = (float)msg->pose.position.y;
    setpoint_z = (float)msg->pose.position.z;   /* positive up */
    /* Extract yaw from quaternion (rotation around Z axis, NED convention).
     * Identity quaternion (w=1) gives yaw=0 — facing North. */
    float qx = (float)msg->pose.orientation.x;
    float qy = (float)msg->pose.orientation.y;
    float qz = (float)msg->pose.orientation.z;
    float qw = (float)msg->pose.orientation.w;
    setpoint_yaw = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz));
    setpoint_received = true;

    ESP_LOGI(TAG, "Setpoint: (%.2f, %.2f, %.2f up)", setpoint_x, setpoint_y, setpoint_z);

    /* In GCS OFFBOARD mode, forward immediately to PX4 */
    if (gcs_control_active && drone_state == DRONE_ARMED) {
        mav_set_position_ned(setpoint_x, setpoint_y, -setpoint_z);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * Timer callback — 100 ms (state and battery publishing)
 * ══════════════════════════════════════════════════════════════════════════ */

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (!timer) return;

    /* Publish state every 2 s so late subscribers (rqt opened after drone boot)
     * see the current state within 2 seconds.  State transitions are now
     * published immediately via publish_state_now() in command_callback, so
     * state_dirty is cleared there; the periodic re-publish here keeps rqt in
     * sync and must be < GCS DRONE_OFFLINE_TIMEOUT_S (3 s). */
    static uint32_t state_tick = 0;
    if (state_dirty || (++state_tick >= 20)) {
        state_dirty = false;
        state_tick  = 0;
        const char *s = state_names[(int)drone_state];
        rosidl_runtime_c__String__assign(&state_pub_msg.data, s);
        RCSOFTCHECK(rcl_publish(&publisher_state, &state_pub_msg, NULL));
    }

    /* Publish battery every 5 s (50 × 100 ms ticks) */
    static uint32_t bat_tick = 0;
    if (++bat_tick >= 50) {
        bat_tick = 0;
        battery_pub_msg.data = battery_remaining_pct;
        RCSOFTCHECK(rcl_publish(&publisher_battery, &battery_pub_msg, NULL));
    }

}

/* ══════════════════════════════════════════════════════════════════════════
 * MAVLink UART RX task — parses incoming PX4 telemetry
 *
 * Runs independently of micro-ROS.  Reads raw bytes from the shared UART
 * (UART_NUM_1) that we also use for TX.  The UART driver's internal RX ring
 * buffer (2 KB) decouples this task from the TX path so there is no locking.
 *
 * Currently handles:
 *   BATTERY_STATUS      — stores battery_remaining_pct (0–100, -1=unknown)
 *   LOCAL_POSITION_NED  — stores px4_pos_x/y/z for home-position capture
 * ══════════════════════════════════════════════════════════════════════════ */

static void mavlink_rx_task_fn(void *arg)
{
    mavlink_message_t rx_msg;
    mavlink_status_t  rx_status;
    uint8_t           byte;

    ESP_LOGI(TAG, "MAVLink RX task started");
    while (true) {
        /* Block up to 100 ms for a byte; loop keeps the task alive when idle */
        int n = uart_read_bytes(UART_NUM_1, &byte, 1, pdMS_TO_TICKS(100));
        if (n <= 0) continue;

        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &rx_msg, &rx_status)) {
            if (rx_msg.msgid == MAVLINK_MSG_ID_BATTERY_STATUS) {
                mavlink_battery_status_t bat;
                mavlink_msg_battery_status_decode(&rx_msg, &bat);
                battery_remaining_pct = bat.battery_remaining;
                ESP_LOGD(TAG, "Battery: %d%%", (int)bat.battery_remaining);
            } else if (rx_msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
                mavlink_local_position_ned_t lpos;
                mavlink_msg_local_position_ned_decode(&rx_msg, &lpos);
                /* NED z is down (positive = down); store altitude as positive-up */
                px4_pos_x = lpos.x;
                px4_pos_y = lpos.y;
                px4_pos_z = -lpos.z;
                px4_pos_valid = true;
            }
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * micro-ROS task
 * ══════════════════════════════════════════════════════════════════════════ */

static void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t  support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options =
        rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    RCCHECK(rmw_uros_options_set_client_key((uint32_t)DRONE_ID, rmw_options));
#endif

    ESP_LOGI(TAG, "uros_task running, agent=%s:%s",
             CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
    ESP_LOGI(TAG, "Waiting for micro-ROS agent...");
    while (rclc_support_init_with_options(&support, 0, NULL,
                                           &init_options, &allocator) != RCL_RET_OK) {
        ESP_LOGW(TAG, "Agent not reachable, retrying in 2 s...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    ESP_LOGI(TAG, "micro-ROS agent connected!");

    rcl_node_t node;
    char node_name[32];
    snprintf(node_name, sizeof(node_name), "esp32_drone_brain_%d", DRONE_ID);
    RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

    /* ── Publishers ──────────────────────────────────────────────────────── */
    RCCHECK(rclc_publisher_init_default(&publisher_state, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_state));

    RCCHECK(rclc_publisher_init_default(&publisher_role, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_role));

    RCCHECK(rclc_publisher_init_default(&publisher_battery, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), topic_battery));

    /* ── Subscribers ─────────────────────────────────────────────────────── */
    RCCHECK(rclc_subscription_init_best_effort(&command_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_gcs_command));

    RCCHECK(rclc_subscription_init_best_effort(&config_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_gcs_config));

    RCCHECK(rclc_subscription_init_best_effort(&control_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
        topic_gcs_control));


    /* ── Message buffers ─────────────────────────────────────────────────── */
    command_msg.data.data = (char *)malloc(64);
    command_msg.data.size = 0; command_msg.data.capacity = 64; command_msg.data.data[0] = '\0';

    config_msg.data.data = (char *)malloc(64);
    config_msg.data.size = 0; config_msg.data.capacity = 64; config_msg.data.data[0] = '\0';

    geometry_msgs__msg__PoseStamped__init(&control_msg);

    state_pub_msg.data.data = (char *)malloc(32);
    state_pub_msg.data.size = 0; state_pub_msg.data.capacity = 32; state_pub_msg.data.data[0] = '\0';

    role_pub_msg.data.data = (char *)malloc(32);
    role_pub_msg.data.size = 0; role_pub_msg.data.capacity = 32; role_pub_msg.data.data[0] = '\0';


    /* Publish initial role — not yet used, placeholder */
    rosidl_runtime_c__String__assign(&role_pub_msg.data, "idle");
    RCSOFTCHECK(rcl_publish(&publisher_role, &role_pub_msg, NULL));

    /* ── Timer + executor (1 timer + 3 subscriptions = 4 handles) ─────── */
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback));


    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &command_sub,
                &command_msg, &command_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &config_sub,
                &config_msg, &config_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &control_sub,
                &control_msg, &control_callback, ON_NEW_DATA));

    /* C2 watchdog state — ping runs in this task to avoid transport race conditions */
    int64_t last_c2_check_ms = now_ms();
    int     c2_failures       = 0;

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));

        int64_t t = now_ms();
        if (t - last_c2_check_ms >= C2_CHECK_INTERVAL_MS) {
            last_c2_check_ms = t;
            if (rmw_uros_ping_agent(C2_PING_TIMEOUT_MS, C2_PING_ATTEMPTS) == RMW_RET_OK) {
                c2_failures = 0;
            } else {
                c2_failures++;
                ESP_LOGW(TAG, "C2 ping failed (%d/%d)", c2_failures, C2_FAIL_THRESHOLD);
                if (c2_failures >= C2_FAIL_THRESHOLD) {
                    c2_failures = 0;
                    if (drone_state == DRONE_DISARMED || drone_state == DRONE_KILLED) {
                        /* Safe to restart — drone is on the ground.
                         * esp_restart gives a clean micro-ROS session so rqt
                         * sees the drone online without needing a power cycle. */
                        ESP_LOGE(TAG, "C2 link lost while disarmed — restarting for clean reconnect");
                        vTaskDelay(pdMS_TO_TICKS(500));
                        esp_restart();
                    } else if (drone_state != DRONE_LANDING) {
                        ESP_LOGE(TAG, "C2 link lost — initiating emergency landing");
                        trigger_eland();
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    RCCHECK(rcl_publisher_fini(&publisher_state,   &node));
    RCCHECK(rcl_publisher_fini(&publisher_role,    &node));
    RCCHECK(rcl_publisher_fini(&publisher_battery, &node));
    RCCHECK(rcl_subscription_fini(&command_sub,      &node));
    RCCHECK(rcl_subscription_fini(&config_sub,        &node));
    RCCHECK(rcl_subscription_fini(&control_sub,       &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Device hostname / DHCP
 * ══════════════════════════════════════════════════════════════════════════ */

static void set_device_hostname_from_drone_id(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) { ESP_LOGW(TAG, "netif not found"); return; }
    char hostname[32];
    snprintf(hostname, sizeof(hostname), "mach-mind-drone-%d", DRONE_ID);
    if (esp_netif_set_hostname(netif, hostname) == ESP_OK)
        ESP_LOGI(TAG, "Hostname: %s", hostname);
}

static void print_current_ip_info(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) return;
    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(netif, &ip) == ESP_OK)
        ESP_LOGI(TAG, "IP: " IPSTR "  GW: " IPSTR "  MASK: " IPSTR,
                 IP2STR(&ip.ip), IP2STR(&ip.gw), IP2STR(&ip.netmask));
}

/* ══════════════════════════════════════════════════════════════════════════
 * app_main
 * ══════════════════════════════════════════════════════════════════════════ */

void app_main(void)
{
  printf("app_main started\r\n"); fflush(stdout);

    /* Initialize the network before starting micro-ROS. */
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    set_device_hostname_from_drone_id();
    /* Keep DHCP enabled. The phone hotspot assigns the ESP32 its IP,
     * gateway and netmask automatically. */
    ESP_LOGI(TAG, "Connecting using hotspot DHCP; agent=%s:%s",
             CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
    print_current_ip_info();
#endif

    /* Build per-drone topic strings */
    snprintf(drone_ns,           sizeof(drone_ns),           "drone_%d",                   DRONE_ID);
    snprintf(topic_gcs_command,  sizeof(topic_gcs_command),   "/gcs/drone_%d/command",      DRONE_ID);
    snprintf(topic_gcs_config,   sizeof(topic_gcs_config),    "/gcs/drone_%d/config",       DRONE_ID);
    snprintf(topic_gcs_control,  sizeof(topic_gcs_control),   "/gcs/drone_%d/control",      DRONE_ID);
    snprintf(topic_state,        sizeof(topic_state),          "/drone_%d/state",            DRONE_ID);
    snprintf(topic_role,         sizeof(topic_role),           "/drone_%d/role",             DRONE_ID);
    snprintf(topic_battery,      sizeof(topic_battery),        "/drone_%d/battery",          DRONE_ID);

    ESP_LOGI(TAG, "==============================");
    ESP_LOGI(TAG, "DRONE ID   : %d",           DRONE_ID);
    ESP_LOGI(TAG, "Namespace  : %s",           drone_ns);
    ESP_LOGI(TAG, "CMD topic  : %s",           topic_gcs_command);
    ESP_LOGI(TAG, "CFG topic  : %s",           topic_gcs_config);
    ESP_LOGI(TAG, "CTL topic  : %s",           topic_gcs_control);
    ESP_LOGI(TAG, "State topic: %s",           topic_state);
    ESP_LOGI(TAG, "Takeoff alt: %.1f m",       MISSION_TAKEOFF_ALT_M);
    ESP_LOGI(TAG, "Hostname   : mach-mind-drone-%d", DRONE_ID);
    ESP_LOGI(TAG, "Network: phone hotspot via DHCP");
    ESP_LOGI(TAG, "==============================");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DRONE_ID_LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = 0, .pull_down_en = 0,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("\r\nDevice Starting — Drone ID %d\r\n", DRONE_ID);

    uart_mavlink_init();
    xTaskCreate(mavlink_rx_task_fn, "mav_rx", 4096, NULL, 4, NULL);

    /* ESP32-C6 has no external PSRAM on the XIAO board.
     * Allocate the micro-ROS task stack from internal SRAM. FreeRTOS stack
     * depth is expressed in StackType_t words, matching CONFIG_MICRO_ROS_APP_STACK. */
    ESP_LOGI(TAG, "Free internal heap: %u B",
             (unsigned)esp_get_free_heap_size());
    BaseType_t uros_created = xTaskCreate(
                micro_ros_task, "uros_task",
                CONFIG_MICRO_ROS_APP_STACK, NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);
    if (uros_created != pdPASS) {
        ESP_LOGE(TAG, "uros_task creation FAILED — insufficient internal RAM");
    } else {
        ESP_LOGI(TAG, "uros_task created OK (stack=%d words, internal SRAM)",
                 CONFIG_MICRO_ROS_APP_STACK);
    }

    while (1) {
        drone_id_led_update();
        send_heartbeat_once();
        printf("\r|STATE:%-14s|BAT:%3d%%|PX4_POS:%s|",
               state_names[(int)drone_state],
               (int)battery_remaining_pct,
               px4_pos_valid ? "OK" : "WAIT");
        fflush(stdout);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}