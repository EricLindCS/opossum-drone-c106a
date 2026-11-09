idf_component_register(
    SRCS
        "main.c"
        "mavlink/mavlink_interface.c"

    INCLUDE_DIRS
        "."
        "mavlink"
        "${PROJECT_DIR}/managed_components/espressif__mavlink/c_library_v2"
        "${PROJECT_DIR}/../../micro_ros_espidf_component/micro_ros_src/src/rcl/rcl/include"

    REQUIRES
        driver
        esp_timer
        esp_system
        esp_netif
        nvs_flash
        micro_ros_espidf_component
        espressif__mavlink
        esp_driver_uart
        esp_driver_gpio

    PRIV_REQUIRES
        esp_timer
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=address-of-packed-member
)