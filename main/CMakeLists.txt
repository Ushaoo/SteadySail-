idf_component_register(
    SRCS "blinker_bridge.c" "control_params.c" "main.c" 
         "imu_driver.c" 
         "balance_controller.c" 
         "motor_control.c"
         "steering_control.c"
    INCLUDE_DIRS "."
    REQUIRES Blinker json
)
