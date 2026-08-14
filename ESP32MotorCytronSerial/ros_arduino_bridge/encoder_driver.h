#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <Arduino.h>

// ESP32 Pin definitions for JGA25-370 Hall Effect Encoders
#define LEFT_ENC_PIN_A  18
#define LEFT_ENC_PIN_B  19
#define RIGHT_ENC_PIN_A 22
#define RIGHT_ENC_PIN_B 23

volatile long left_enc_pos = 0;
volatile long right_enc_pos = 0;

void IRAM_ATTR leftEncoderISR() {
    if (digitalRead(LEFT_ENC_PIN_A) == digitalRead(LEFT_ENC_PIN_B)) {
        left_enc_pos++;
    } else {
        left_enc_pos--;
    }
}

void IRAM_ATTR rightEncoderISR() {
    if (digitalRead(RIGHT_ENC_PIN_A) == digitalRead(RIGHT_ENC_PIN_B)) {
        right_enc_pos--;
    } else {
        right_enc_pos++;
    }
}

void initEncoders() {
    pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
    pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), rightEncoderISR, CHANGE);
}

long readEncoder(int i) {
    if (i == 0) return left_enc_pos;
    else return right_enc_pos;
}

void resetEncoder(int i) {
    if (i == 0) left_enc_pos = 0;
    else right_enc_pos = 0;
}

void resetEncoders() {
    left_enc_pos = 0;
    right_enc_pos = 0;
}

#endif
