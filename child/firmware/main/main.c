
/* main.c — ArUco vision + pose publisher (micro-ROS)
 *
 * Published:
 *   /drone_{ID}/aruco_pose
 *       geometry_msgs/PoseStamped
 *       x forward, y right, z up, meters
 *
 *   /drone_{ID}/camera/image_raw
 *       sensor_msgs/Image
 *       mono8 80x60 @ 5 FPSa
 *
 * Networking:
 *   Uses DHCP, matching the known-working micro-ROS firmware.
 *
 * Task allocation:
 *   micro-ROS task uses xTaskCreate() and internal task memory.
 *   Camera frame buffers remain in PSRAM.
 */

#ifndef __STDC_WANT_LIB_EXT1__
#define __STDC_WANT_LIB_EXT1__ 0
#endif

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdbool.h>
#include <sys/types.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_netif.h"

#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <rosidl_runtime_c/string_functions.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include <sensor_msgs/msg/image.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <uros_network_interfaces.h>

#include "aruco_task.h"

#define CAMERA_MODEL_XIAO_ESP32S3 1
#include "boards.h"

static const char *TAG = "drone";

/* ── Configuration ───────────────────────────────────────────────────── */

#define DRONE_ID            1
#define VISION_TIMEOUT_MS   1500
#define CAMERA_STREAM       1

/* ── Error handling ──────────────────────────────────────────────────── */

#define RCCHECK(fn)                                                   \
    {                                                                 \
        rcl_ret_t _rc = (fn);                                         \
        if (_rc != RCL_RET_OK) {                                      \
            printf("Failed line %d: %d. Aborting.\n",                 \
                   __LINE__, (int)_rc);                               \
            vTaskDelete(NULL);                                        \
        }                                                             \
    }

#define RCSOFTCHECK(fn)                                                \
    {                                                                  \
        rcl_ret_t _rc = (fn);                                          \
        if (_rc != RCL_RET_OK) {                                       \
            printf("Soft fail line %d: %d.\n",                         \
                   __LINE__, (int)_rc);                                \
        }                                                              \
    }

/* ── Shared with aruco_task.cpp ─────────────────────────────────────── */

volatile bool vision_enabled = true;

volatile float vp_x = 0.0f;
volatile float vp_y = 0.0f;
volatile float vp_z = 0.0f;

volatile float vp_qx = 0.0f;
volatile float vp_qy = 0.0f;
volatile float vp_qz = 0.0f;
volatile float vp_qw = 1.0f;

volatile bool vision_pose_valid = false;
volatile int64_t last_vision_pose_ms = 0;

/* ── Camera state ────────────────────────────────────────────────────── */

static bool camera_hw_ok = false;
static bool camera_sw_rotate = false;

/* ── micro-ROS entities ─────────────────────────────────────────────── */

static rcl_publisher_t publisher_image;
static bool publisher_image_ok = false;

static rcl_publisher_t publisher_pose;

static sensor_msgs__msg__Image img_msg;
static geometry_msgs__msg__PoseStamped pose_msg;

static struct timespec ts;

static char topic_image[64];
static char topic_pose[64];
static char topic_camera_frame[64];

/* ── 100 ms timer callback ───────────────────────────────────────────── */

static void timer_callback(
    rcl_timer_t *timer,
    int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    if (timer == NULL) {
        return;
    }

    int64_t now = esp_timer_get_time() / 1000;

    /* Invalidate stale vision measurements. */
    if (vision_pose_valid &&
        (now - last_vision_pose_ms) > VISION_TIMEOUT_MS) {

        vision_pose_valid = false;
        ESP_LOGW(TAG, "Vision pose timeout");
    }

    /* ── Publish pose when a new measurement is available ─────────── */

    static int64_t last_pub_ms = 0;

    if (vision_pose_valid &&
        last_vision_pose_ms != last_pub_ms) {

        last_pub_ms = last_vision_pose_ms;

        clock_gettime(CLOCK_REALTIME, &ts);

        pose_msg.header.stamp.sec = ts.tv_sec;
        pose_msg.header.stamp.nanosec = ts.tv_nsec;

        pose_msg.pose.position.x = vp_x;
        pose_msg.pose.position.y = vp_y;
        pose_msg.pose.position.z = vp_z;

        pose_msg.pose.orientation.x = vp_qx;
        pose_msg.pose.orientation.y = vp_qy;
        pose_msg.pose.orientation.z = vp_qz;
        pose_msg.pose.orientation.w = vp_qw;

        RCSOFTCHECK(
            rcl_publish(&publisher_pose, &pose_msg, NULL)
        );
    }

#if CAMERA_STREAM

    /*
     * Timer runs at 10 Hz.
     * Publish every second callback = 5 FPS.
     *
     * Camera frame:
     *   160x120 grayscale
     *
     * Published frame:
     *   80x60 mono8
     */

    static uint8_t img_skip = 0;

    if (publisher_image_ok && ++img_skip >= 2) {

        img_skip = 0;

        camera_fb_t *pic = esp_camera_fb_get();

        if (pic != NULL) {

            const size_t output_size = 80 * 60;

            if (pic->len >= 160 * 120 &&
                img_msg.data.capacity >= output_size) {

                clock_gettime(CLOCK_REALTIME, &ts);

                img_msg.header.stamp.sec = ts.tv_sec;
                img_msg.header.stamp.nanosec = ts.tv_nsec;

                img_msg.header.frame_id =
                    micro_ros_string_utilities_set(
                        img_msg.header.frame_id,
                        topic_camera_frame
                    );

                img_msg.width = 80;
                img_msg.height = 60;
                img_msg.step = 80;

                img_msg.encoding =
                    micro_ros_string_utilities_set(
                        img_msg.encoding,
                        "mono8"
                    );

                img_msg.is_bigendian = 0;
                img_msg.data.size = output_size;

                const uint8_t *src =
                    (const uint8_t *)pic->buf;

                uint8_t *dst = img_msg.data.data;

                for (int y = 0; y < 60; y++) {

                    for (int x = 0; x < 80; x++) {

                        int src_y = camera_sw_rotate
                            ? (119 - y * 2)
                            : (y * 2);

                        int src_x = x * 2;

                        dst[y * 80 + x] =
                            src[src_y * 160 + src_x];
                    }
                }

                RCSOFTCHECK(
                    rcl_publish(&publisher_image, &img_msg, NULL)
                );
            }

            esp_camera_fb_return(pic);

        } else {

            ESP_LOGW(TAG, "Camera capture failed");
        }
    }

#endif
}

/* ── micro-ROS task ──────────────────────────────────────────────────── */

static void micro_ros_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "micro-ROS task started");

    rcl_allocator_t allocator = rcl_get_default_allocator();

    rclc_support_t support;

    rcl_init_options_t init_options =
        rcl_get_zero_initialized_init_options();

    RCCHECK(
        rcl_init_options_init(&init_options, allocator)
    );

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE

    rmw_init_options_t *rmw_options =
        rcl_init_options_get_rmw_init_options(&init_options);

    RCCHECK(
        rmw_uros_options_set_udp_address(
            CONFIG_MICRO_ROS_AGENT_IP,
            CONFIG_MICRO_ROS_AGENT_PORT,
            rmw_options
        )
    );

    RCCHECK(
        rmw_uros_options_set_client_key(
            (uint32_t)DRONE_ID,
            rmw_options
        )
    );

#endif

    ESP_LOGI(
        TAG,
        "Waiting for micro-ROS agent %s:%d",
        CONFIG_MICRO_ROS_AGENT_IP,
        (int)CONFIG_MICRO_ROS_AGENT_PORT
    );

    /* ── Wait for agent ───────────────────────────────────────────── */

    while (true) {

        rcl_ret_t ret =
            rclc_support_init_with_options(
                &support,
                0,
                NULL,
                &init_options,
                &allocator
            );

        if (ret == RCL_RET_OK) {

            ESP_LOGI(TAG, "micro-ROS agent connected!");
            break;
        }

        ESP_LOGE(
            TAG,
            "micro-ROS support init failed: %d",
            (int)ret
        );

        if (rcl_error_is_set()) {

            ESP_LOGE(
                TAG,
                "RCL error: %s",
                rcl_get_error_string().str
            );

            rcl_reset_error();
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    /* ── Initialize node ───────────────────────────────────────────── */

    rcl_node_t node;

    char node_name[32];

    snprintf(
        node_name,
        sizeof(node_name),
        "esp32_drone_vision_%d",
        DRONE_ID
    );

    RCCHECK(
        rclc_node_init_default(
            &node,
            node_name,
            "",
            &support
        )
    );

    ESP_LOGI(TAG, "Node initialized: %s", node_name);

    /* ── Image publisher ───────────────────────────────────────────── */

#if CAMERA_STREAM

    publisher_image_ok =
        (rclc_publisher_init_default(
            &publisher_image,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                sensor_msgs,
                msg,
                Image
            ),
            topic_image
        ) == RCL_RET_OK);

    if (publisher_image_ok) {

        ESP_LOGI(
            TAG,
            "Image publisher initialized: %s",
            topic_image
        );

    } else {

        ESP_LOGW(
            TAG,
            "Image publisher initialization failed"
        );
    }

#endif

    /* ── Pose publisher ────────────────────────────────────────────── */

    RCCHECK(
        rclc_publisher_init_default(
            &publisher_pose,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                geometry_msgs,
                msg,
                PoseStamped
            ),
            topic_pose
        )
    );

    ESP_LOGI(
        TAG,
        "Pose publisher initialized: %s",
        topic_pose
    );

    /* ── Initialize pose message ───────────────────────────────────── */

    geometry_msgs__msg__PoseStamped__init(&pose_msg);

    rosidl_runtime_c__String__assign(
        &pose_msg.header.frame_id,
        "aruco_map"
    );

    /* ── Allocate image message memory ─────────────────────────────── */

    static micro_ros_utilities_memory_conf_t conf = {};

    conf.max_string_capacity = 50;
    conf.max_ros2_type_sequence_capacity = 5;
    conf.max_basic_type_sequence_capacity = 5;

    micro_ros_utilities_memory_rule_t const rules[] = {

        {"header.frame_id", 40},
        {"format", 5},
        {"encoding", 10},
        {"data", 19200},

    };

    conf.rules = rules;

    conf.n_rules =
        sizeof(rules) / sizeof(rules[0]);

    bool image_memory_ok =
        micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                sensor_msgs,
                msg,
                Image
            ),
            &img_msg,
            conf
        );

    if (!image_memory_ok) {

        ESP_LOGE(
            TAG,
            "Image message memory allocation failed"
        );

        publisher_image_ok = false;

    } else {

        ESP_LOGI(
            TAG,
            "Image message memory allocated"
        );
    }

    /* ── Timer ─────────────────────────────────────────────────────── */

    rcl_timer_t timer;

    RCCHECK(
        rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(100),
            timer_callback
        )
    );

    /* ── Executor ──────────────────────────────────────────────────── */

    rclc_executor_t executor;

    RCCHECK(
        rclc_executor_init(
            &executor,
            &support.context,
            1,
            &allocator
        )
    );

    RCCHECK(
        rclc_executor_add_timer(
            &executor,
            &timer
        )
    );

    ESP_LOGI(TAG, "micro-ROS executor initialized");

    /* ── Start ArUco detector after micro-ROS initialization ───────── */

    if (camera_hw_ok) {

        ESP_LOGI(TAG, "Starting ArUco detection task");

        aruco_task_start();

    } else {

        ESP_LOGW(
            TAG,
            "Camera unavailable; ArUco detection disabled"
        );
    }

    ESP_LOGI(TAG, "Entering micro-ROS spin loop");

    /* ── Main spin loop ────────────────────────────────────────────── */

    while (true) {

        rclc_executor_spin_some(
            &executor,
            RCL_MS_TO_NS(50)
        );

        usleep(10000);
    }
}

/* ── app_main ────────────────────────────────────────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "Starting drone vision firmware");

    /* ── Network initialization ────────────────────────────────────── */

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || \
    defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)

    /*
     * IMPORTANT:
     * Use DHCP, just like the known-working firmware.
     *
     * Do not stop DHCP or assign a static IP here.
     * The ESP32 and micro-ROS agent must be reachable
     * on the same network.
     */

    ESP_ERROR_CHECK(
        uros_network_interface_initialize()
    );

    esp_netif_t *netif =
        esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

    if (netif != NULL) {

        char hostname[32];

        snprintf(
            hostname,
            sizeof(hostname),
            "mach-mind-drone-%d",
            DRONE_ID
        );

        esp_netif_set_hostname(netif, hostname);

        ESP_LOGI(
            TAG,
            "Hostname configured: %s",
            hostname
        );

        esp_netif_ip_info_t ip_info;

        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {

            ESP_LOGI(
                TAG,
                "DHCP IP: " IPSTR,
                IP2STR(&ip_info.ip)
            );

            ESP_LOGI(
                TAG,
                "Gateway: " IPSTR,
                IP2STR(&ip_info.gw)
            );
        }

    } else {

        ESP_LOGW(
            TAG,
            "WIFI_STA_DEF netif not found"
        );
    }

    ESP_LOGI(
        TAG,
        "Using micro-ROS agent %s:%d",
        CONFIG_MICRO_ROS_AGENT_IP,
        (int)CONFIG_MICRO_ROS_AGENT_PORT
    );

#else

    ESP_LOGE(
        TAG,
        "No supported micro-ROS network interface configured"
    );

#endif

    /* ── Camera initialization ─────────────────────────────────────── */

    camera_config_t camera_config = {

        .pin_pwdn = CAMERA_PIN_PWDN,
        .pin_reset = CAMERA_PIN_RESET,

        .pin_xclk = CAMERA_PIN_XCLK,

        .pin_sccb_sda = CAMERA_PIN_SIOD,
        .pin_sccb_scl = CAMERA_PIN_SIOC,

        .pin_d7 = CAMERA_PIN_D7,
        .pin_d6 = CAMERA_PIN_D6,
        .pin_d5 = CAMERA_PIN_D5,
        .pin_d4 = CAMERA_PIN_D4,
        .pin_d3 = CAMERA_PIN_D3,
        .pin_d2 = CAMERA_PIN_D2,
        .pin_d1 = CAMERA_PIN_D1,
        .pin_d0 = CAMERA_PIN_D0,

        .pin_vsync = CAMERA_PIN_VSYNC,
        .pin_href = CAMERA_PIN_HREF,
        .pin_pclk = CAMERA_PIN_PCLK,

        .xclk_freq_hz = 20000000,

        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_GRAYSCALE,

        .frame_size = FRAMESIZE_QQVGA,

        .jpeg_quality = 63,

        .fb_count = 8,

        .fb_location = CAMERA_FB_IN_PSRAM,

        .grab_mode = CAMERA_GRAB_LATEST,
    };

    ESP_LOGI(TAG, "Initializing camera");

    esp_err_t camera_ret =
        esp_camera_init(&camera_config);

    if (camera_ret != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Camera initialization failed: %s",
            esp_err_to_name(camera_ret)
        );

        camera_hw_ok = false;

    } else {

        sensor_t *s = esp_camera_sensor_get();

        if (s != NULL) {

            bool ov3660 =
                (s->id.PID == OV3660_PID);

            s->set_gain_ctrl(s, 0);
            s->set_agc_gain(s, 1);

            s->set_exposure_ctrl(s, 0);

            s->set_aec_value(
                s,
                ov3660 ? 300 : 400
            );

            if (ov3660) {

                s->set_denoise(s, 0);
                s->set_contrast(s, 3);
                s->set_sharpness(s, 3);

            } else {

                s->set_contrast(s, 2);
                s->set_sharpness(s, 2);
            }

            s->set_bpc(s, 1);
            s->set_wpc(s, 1);

            s->set_vflip(s, 1);
            s->set_hmirror(s, 1);

            camera_sw_rotate = ov3660;

            ESP_LOGI(
                TAG,
                "Camera sensor: %s",
                ov3660 ? "OV3660" : "OV2640"
            );
        }

        camera_hw_ok = true;

        ESP_LOGI(TAG, "Camera initialized successfully");
    }

    /* ── Topic names ───────────────────────────────────────────────── */

    snprintf(
        topic_image,
        sizeof(topic_image),
        "/drone_%d/camera/image_raw",
        DRONE_ID
    );

    snprintf(
        topic_pose,
        sizeof(topic_pose),
        "/drone_%d/aruco_pose",
        DRONE_ID
    );

    snprintf(
        topic_camera_frame,
        sizeof(topic_camera_frame),
        "drone_%d/camera",
        DRONE_ID
    );

    /* ── Memory diagnostics ────────────────────────────────────────── */

    ESP_LOGI(
        TAG,
        "Free PSRAM: %u bytes",
        (unsigned)heap_caps_get_free_size(
            MALLOC_CAP_SPIRAM
        )
    );

    ESP_LOGI(
        TAG,
        "Free internal heap: %u bytes",
        (unsigned)heap_caps_get_free_size(
            MALLOC_CAP_INTERNAL
        )
    );

    /* ── Start micro-ROS task ──────────────────────────────────────── */

    ESP_LOGI(TAG, "Creating micro-ROS task");

    /*
     * Use xTaskCreate(), matching the known-working firmware.
     *
     * Do not put the micro-ROS task stack in PSRAM.
     * The task needs reliable access to internal memory
     * for networking and middleware operations.
     */

    BaseType_t task_result = xTaskCreate(
        micro_ros_task,
        "uros_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL
    );

    if (task_result != pdPASS) {

        ESP_LOGE(
            TAG,
            "micro-ROS task creation FAILED"
        );

    } else {

        ESP_LOGI(
            TAG,
            "micro-ROS task created successfully"
        );
    }
}
