#ifndef _MAIN_H_
#define _MAIN_H_

#include "zf_common_headfile.h"
#include "img_process.h"
#include "race_autotune.h"

/* Version-1 straight-line checkout controls. */
#define CAR_START_KEY                  (KEY_1)
#define CAR_KEY_SCAN_PERIOD_MS         (10)
#define MOTOR_CONTROL_PERIOD_MS        (10)
#define MOTOR_PWM_FREQ_HZ              (17000)
#define STRAIGHT_TEST_SPEED_TARGET     (15.0f)

/* Encoder pins. Forward motion must produce positive counts on both sides. */
#define ENCODER_1                      (QTIMER1_ENCODER1)
#define ENCODER_1_A                    (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_B                    (QTIMER1_ENCODER1_CH2_C1)
#define ENCODER_2                      (QTIMER1_ENCODER2)
#define ENCODER_2_A                    (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_B                    (QTIMER1_ENCODER2_CH2_C24)

/* Motor pins and forward levels must be verified with the wheels raised. */
#define MOTOR1_PWM                     (PWM2_MODULE0_CHA_C6)
#define MOTOR1_DIR                     (C7)
#define MOTOR1_FORWARD_DIR_LEVEL       (GPIO_HIGH)
#define MOTOR2_PWM                     (PWM2_MODULE1_CHA_C8)
#define MOTOR2_DIR                     (C9)
#define MOTOR2_FORWARD_DIR_LEVEL       (GPIO_HIGH)

/* Servo calibration copied from the teaching example. */
#define MOTOR_SERVO_PWM                (PWM4_MODULE2_CHA_C30)
#define MOTOR_SERVO_FREQ               (50)
#define MOTOR_SERVO_L_MAX              (100)
#define MOTOR_SERVO_R_MAX              (85)
#define MOTOR_SERVO_DUTY(x)            ((float)PWM_DUTY_MAX / (1000.0f / (float)MOTOR_SERVO_FREQ) * (0.5f + (float)(x) / 90.0f))

#if (MOTOR_SERVO_FREQ < 50 || MOTOR_SERVO_FREQ > 300)
#error "MOTOR_SERVO_FREQ outside supported range"
#endif

extern const float servo_motor_duty_middle;
extern volatile uint8 car_run_enabled;

#endif
