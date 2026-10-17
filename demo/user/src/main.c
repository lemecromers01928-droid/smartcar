#include "main.h"

/*
 * Version 1: low-speed straight-line chassis checkout.
 *
 * KEY1 (the SeekFree library default is C15) toggles RUN/STOP. The car is
 * always stopped after reset. Camera steering is intentionally deferred to
 * the next version so motor direction, encoder direction and servo centre can
 * be verified independently and safely.
 */

const float servo_motor_duty_middle =
    (MOTOR_SERVO_R_MAX + MOTOR_SERVO_L_MAX) / 2.0f;

volatile uint8 car_run_enabled = 0;

static void straight_test_init(void)
{
    /* Keep both motors disabled until the operator presses KEY1. */
    gpio_init(MOTOR1_DIR, GPO, MOTOR1_FORWARD_DIR_LEVEL, GPO_PUSH_PULL);
    gpio_init(MOTOR2_DIR, GPO, MOTOR2_FORWARD_DIR_LEVEL, GPO_PUSH_PULL);
    pwm_init(MOTOR1_PWM, MOTOR_PWM_FREQ_HZ, 0);
    pwm_init(MOTOR2_PWM, MOTOR_PWM_FREQ_HZ, 0);

    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);

    /* Centre the steering before motor control is allowed to run. */
    pwm_init(MOTOR_SERVO_PWM, MOTOR_SERVO_FREQ,
             MOTOR_SERVO_DUTY(servo_motor_duty_middle));

    key_init(CAR_KEY_SCAN_PERIOD_MS);
    My_Pid_Init();
    speed_target = 0.0f;

    pit_ms_init(PIT_CH0, MOTOR_CONTROL_PERIOD_MS);
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    system_delay_ms(300);

    straight_test_init();
    interrupt_global_enable(0);

    while (1)
    {
        key_scanner();
        if (KEY_SHORT_PRESS == key_get_state(CAR_START_KEY))
        {
            car_run_enabled = (uint8)!car_run_enabled;
            speed_target = car_run_enabled ? STRAIGHT_TEST_SPEED_TARGET : 0.0f;
            key_clear_state(CAR_START_KEY);
        }

        system_delay_ms(CAR_KEY_SCAN_PERIOD_MS);
    }
}
