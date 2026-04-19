idf_component_register(
    SRCS "main.c" 
         "imu_driver.c" 
         "balance_controller.c" 
         "motor_control.c"
         "steering_control.c"
    INCLUDE_DIRS "."
)
