#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

// BTS7960 / Cytron Driver Pins on ESP32
#define LEFT_MOTOR_PWM_F  25
#define LEFT_MOTOR_PWM_R  26
#define RIGHT_MOTOR_PWM_F 32
#define RIGHT_MOTOR_PWM_R 33

#define PWM_FREQ         20000
#define PWM_RESOLUTION   8

#define CH_LEFT_F        0
#define CH_LEFT_R        1
#define CH_RIGHT_F       2
#define CH_RIGHT_R       3

void initMotorController() {
    ledcSetup(CH_LEFT_F, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_LEFT_R, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_RIGHT_F, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_RIGHT_R, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(LEFT_MOTOR_PWM_F, CH_LEFT_F);
    ledcAttachPin(LEFT_MOTOR_PWM_R, CH_LEFT_R);
    ledcAttachPin(RIGHT_MOTOR_PWM_F, CH_RIGHT_F);
    ledcAttachPin(RIGHT_MOTOR_PWM_R, CH_RIGHT_R);
}

void setMotorSpeed(int i, int spd) {
    int pwm_val = constrain(abs(spd), 0, 255);
    if (i == 0) { // Left Motor
        if (spd >= 0) {
            ledcWrite(CH_LEFT_F, pwm_val);
            ledcWrite(CH_LEFT_R, 0);
        } else {
            ledcWrite(CH_LEFT_F, 0);
            ledcWrite(CH_LEFT_R, pwm_val);
        }
    } else { // Right Motor
        if (spd >= 0) {
            ledcWrite(CH_RIGHT_F, pwm_val);
            ledcWrite(CH_RIGHT_R, 0);
        } else {
            ledcWrite(CH_RIGHT_F, 0);
            ledcWrite(CH_RIGHT_R, pwm_val);
        }
    }
}

void setMotorSpeeds(int leftSpd, int rightSpd) {
    setMotorSpeed(0, leftSpd);
    setMotorSpeed(1, rightSpd);
}

#endif
