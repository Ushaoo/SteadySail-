idf_component_register(
    SRCS "fusion.c" "gps_nmea.c" "anchor_control.c" "rc_input.c" "bno055_driver.c" "blinker_bridge.c" "control_params.c" "main.c"
         "imu_driver.c"
         "balance_controller.c"
         "motor_control.c"
         "steering_control.c"
         "fusion.c"
    INCLUDE_DIRS "."
    REQUIRES Blinker json nvs_flash driver esp_timer
)
