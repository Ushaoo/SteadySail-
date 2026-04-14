#include <stdio.h>

void run_esp32s3_test(void);
void run_pwm_breathe(void);
void run_pwm_keyboard_control(void);
int pwm_mode_selector(void);
void run_pwm_capture_angle(void);
void run_pwm_pid_angle_control(void);


void app_main(void)
{
    run_pwm_pid_angle_control();


    // int mode = pwm_mode_selector();
    // if (mode == 1) {
    //     run_pwm_breathe();
    // } else {
    //     run_pwm_keyboard_control();
    // }
}
