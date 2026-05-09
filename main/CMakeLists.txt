idf_component_register(
    SRCS "rc_input.c" "bno055_driver.c" "blinker_bridge.c" "control_params.c" "main.c"
         "imu_driver.c"
         "balance_controller.c"
         "motor_control.c"
         "steering_control.c"
    INCLUDE_DIRS "."
    REQUIRES Blinker json nvs_flash
)
