#pragma once

#include <Arduino.h>

namespace side_arm {

// Limit switches (active LOW with INPUT_PULLUP)
constexpr uint8_t LIMIT_SW_1 = 25;
constexpr uint8_t LIMIT_SW_2 = 26;
constexpr uint8_t LIMIT_SW_3 = 27;

// Stepper drivers (A4988)
constexpr uint8_t STEPPER1_STEP = 18;
constexpr uint8_t STEPPER1_DIR  = 19;
constexpr uint8_t STEPPER2_STEP = 21;
constexpr uint8_t STEPPER2_DIR  = 22;
constexpr uint8_t STEPPER_ENABLE = 12;  // LOW = enabled (shared)

// DC motor (BTS7960 dual PWM + shared EN)
constexpr uint8_t DC_R_PWM = 14;
constexpr uint8_t DC_L_PWM = 15;
constexpr uint8_t DC_EN    = 4;   // tie both EN pins to GPIO4

// Motion defaults
constexpr float DEFAULT_MAX_SPEED    = 800.0F;   // steps / s
constexpr float DEFAULT_ACCELERATION = 200.0F;   // steps / s^2
constexpr float HOMING_SPEED         = 150.0F;

// DC PWM
constexpr uint8_t DC_R_PWM_CHANNEL = 0;
constexpr uint8_t DC_L_PWM_CHANNEL = 1;
constexpr uint32_t DC_PWM_FREQ = 5000;
constexpr uint8_t DC_PWM_RES_BITS = 10;  // 0-1023

}  // namespace side_arm

