idf_component_register(
    SRCS
        "main.c"
        "aruco_task.cpp"
    LDFRAGMENTS "linker.lf"
    INCLUDE_DIRS
        "."
    REQUIRES
        mavlink
        esp32-camera
        esp_driver_uart
        esp_driver_gpio
        micro_ros_espidf_component
        espressif__opencv
    PRIV_REQUIRES
        esp_timer
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    CAMERA_MODEL_XIAO_ESP32S3
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=address-of-packed-member
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-enum-enum-conversion>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-enum-float-conversion>
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=address-of-packed-member
)
