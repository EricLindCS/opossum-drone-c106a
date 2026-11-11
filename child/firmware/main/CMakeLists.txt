idf_component_register(
    SRCS
        "main.c"
        "mavlink/mavlink_interface.c"
    INCLUDE_DIRS
        "."
        "mavlink"
    REQUIRES
        mavlink
        esp_driver_uart
        esp_driver_gpio
    PRIV_REQUIRES
        esp_timer
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=address-of-packed-member
)