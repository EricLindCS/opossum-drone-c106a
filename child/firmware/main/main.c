#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "mavlink/mavlink_interface.h"
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/vector3.h>

#include "freertos/semphr.h"


#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

// PX4 telemetry shared between UART and micro-ROS tasks
static px4_telemetry_t shared_telemetry;
static SemaphoreHandle_t telemetry_mutex;

// ROS 2 publishers
static rcl_publisher_t heartbeat_publisher;
static rcl_publisher_t attitude_publisher;
static rcl_publisher_t position_publisher;
static rcl_publisher_t velocity_publisher;
static rcl_publisher_t battery_publisher;

// ROS 2 messages
static std_msgs__msg__Bool heartbeat_msg;
static geometry_msgs__msg__Vector3 attitude_msg;
static geometry_msgs__msg__Vector3 position_msg;
static geometry_msgs__msg__Vector3 velocity_msg;
static std_msgs__msg__Float32 battery_msg;


void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		printf("Publishing: %d\n", (int) msg.data);
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		msg.data++;
	}
}

void telemetry_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;

    if (timer == NULL || telemetry_mutex == NULL) {
        return;
    }

    px4_telemetry_t snapshot = {0};

    if (xSemaphoreTake(telemetry_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        snapshot = shared_telemetry;
        xSemaphoreGive(telemetry_mutex);
    } else {
        return;
    }

    heartbeat_msg.data = snapshot.heartbeat_received;
    RCSOFTCHECK(rcl_publish(&heartbeat_publisher, &heartbeat_msg, NULL));

    if (snapshot.attitude_received) {
        attitude_msg.x = snapshot.roll;
        attitude_msg.y = snapshot.pitch;
        attitude_msg.z = snapshot.yaw;

        RCSOFTCHECK(rcl_publish(
            &attitude_publisher, &attitude_msg, NULL
        ));
    }

    if (snapshot.position_received) {
        position_msg.x = snapshot.x;
        position_msg.y = snapshot.y;
        position_msg.z = snapshot.z;

        RCSOFTCHECK(rcl_publish(
            &position_publisher, &position_msg, NULL
        ));

        velocity_msg.x = snapshot.vx;
        velocity_msg.y = snapshot.vy;
        velocity_msg.z = snapshot.vz;

        RCSOFTCHECK(rcl_publish(
            &velocity_publisher, &velocity_msg, NULL
        ));
    }

    if (snapshot.battery_received) {
        battery_msg.data = snapshot.battery_voltage;

        RCSOFTCHECK(rcl_publish(
            &battery_publisher, &battery_msg, NULL
        ));
    }
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options =
        rcl_get_zero_initialized_init_options();

    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options =
        rcl_init_options_get_rmw_init_options(&init_options);

    RCCHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP,
        CONFIG_MICRO_ROS_AGENT_PORT,
        rmw_options
    ));
#endif

    RCCHECK(rclc_support_init_with_options(
        &support, 0, NULL, &init_options, &allocator
    ));

    rcl_node_t node;

    RCCHECK(rclc_node_init_default(
        &node,
        "esp32_px4_bridge",
        "",
        &support
    ));

    // Original diagnostic publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "freertos_int32_publisher"
    ));

    // PX4 heartbeat received
    RCCHECK(rclc_publisher_init_default(
        &heartbeat_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "child/heartbeat_received"
    ));

        // PX4 attitude: roll, pitch, yaw
        rcl_ret_t attitude_rc = rclc_publisher_init_default(
        &attitude_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "child/attitude"
    );

    if (attitude_rc != RCL_RET_OK) {
        rcl_error_string_t error = rcl_get_error_string();

        ESP_LOGE("MICROROS",
            "Attitude publisher failed: code=%d, message=%s",
            (int)attitude_rc,
            error.str
        );

        rcl_reset_error();
        vTaskDelete(NULL);
    }

    // PX4 local position in NED
    RCCHECK(rclc_publisher_init_default(
        &position_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "child/local_position"
    ));

    // PX4 velocity in NED
    RCCHECK(rclc_publisher_init_default(
        &velocity_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "child/velocity"
    ));

    // PX4 battery voltage
    RCCHECK(rclc_publisher_init_default(
        &battery_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "child/battery_voltage"
    ));

    // Original test timer: 1 second
    rcl_timer_t test_timer;

    RCCHECK(rclc_timer_init_default(
        &test_timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback
    ));

    // PX4 telemetry timer: 10 Hz
    rcl_timer_t telemetry_timer;

    RCCHECK(rclc_timer_init_default(
        &telemetry_timer,
        &support,
        RCL_MS_TO_NS(30),
        telemetry_timer_callback
    ));

    // Two timer handles
    rclc_executor_t executor;

    RCCHECK(rclc_executor_init(
        &executor,
        &support.context,
        2,
        &allocator
    ));

    RCCHECK(rclc_executor_add_timer(&executor, &test_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &telemetry_timer));

    msg.data = 0;

    ESP_LOGI("MICROROS", "PX4 telemetry publishers initialized");

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        usleep(1000);
    }
}

#define PX4_UART       UART_NUM_1
#define PX4_UART_TX    GPIO_NUM_16
#define PX4_UART_RX    GPIO_NUM_17
#define PX4_UART_BAUD  115200

void px4_uart_task(void *arg)
{
    uart_config_t uart_config = {
        .baud_rate = PX4_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(
        PX4_UART, 2048, 0, 0, NULL, 0
    ));

    ESP_ERROR_CHECK(uart_param_config(PX4_UART, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(
        PX4_UART,
        PX4_UART_TX,
        PX4_UART_RX,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    mavlink_interface_init();

    ESP_LOGI("PX4_UART", "Listening for PX4 MAVLink");

    uint8_t data[128];
    px4_telemetry_t telemetry;

    TickType_t last_log = xTaskGetTickCount();

    while (1) {
        int len = uart_read_bytes(
            PX4_UART,
            data,
            sizeof(data),
            pdMS_TO_TICKS(100)
        );

        for (int i = 0; i < len; i++) {
            mavlink_interface_parse_byte(data[i]);
        }

        // Copy parsed MAVLink telemetry into shared ROS data
        px4_telemetry_t latest_telemetry;

        mavlink_interface_get_telemetry(&latest_telemetry);

        if (xSemaphoreTake(telemetry_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            shared_telemetry = latest_telemetry;
            xSemaphoreGive(telemetry_mutex);
        }

        if ((xTaskGetTickCount() - last_log) >= pdMS_TO_TICKS(1000)) {
            mavlink_interface_get_telemetry(&telemetry);

            ESP_LOGI("PX4_UART",
                "Heartbeat: %d | Attitude: %d | Position: %d | Battery: %d",
                telemetry.heartbeat_received,
                telemetry.attitude_received,
                telemetry.position_received,
                telemetry.battery_received
            );

            if (telemetry.attitude_received) {
                ESP_LOGI("PX4_UART",
                    "RPY: %.2f %.2f %.2f rad",
                    telemetry.roll,
                    telemetry.pitch,
                    telemetry.yaw
                );
            }

            if (telemetry.position_received) {
                ESP_LOGI("PX4_UART",
                    "Position NED: %.2f %.2f %.2f m | Velocity: %.2f %.2f %.2f m/s",
                    telemetry.x, telemetry.y, telemetry.z,
                    telemetry.vx, telemetry.vy, telemetry.vz
                );
            }

            if (telemetry.battery_received) {
                ESP_LOGI("PX4_UART",
                    "Battery: %.2f V | Remaining: %d%%",
                    telemetry.battery_voltage,
                    telemetry.battery_remaining
                );
            }

            last_log = xTaskGetTickCount();
        }
    }
}

void app_main(void)
{

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    telemetry_mutex = xSemaphoreCreateMutex();

    if (telemetry_mutex == NULL) {
        ESP_LOGE("MAIN", "Failed to create telemetry mutex");
        return;
    }

    memset(&shared_telemetry, 0, sizeof(shared_telemetry));
    shared_telemetry.battery_remaining = -1;
    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);

    xTaskCreate(
        px4_uart_task,
        "px4_uart_task",
        4096,
        NULL,
        5,
        NULL
    );

}

