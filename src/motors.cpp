/*
 * =====================================================
 * File: motors.cpp
 * Purpose: L298N Mini (MX1508) Dual H-Bridge motor driver
 *          implementation using ESP32 LEDC hardware PWM
 * =====================================================
 * Compatible with both ESP32 Arduino Core 2.x and 3.x.
 * 4-pin direct PWM control (IN1, IN2, IN3, IN4).
 * Controls speed and direction of two N20 gear motors.
 * =====================================================
 */

#include "motors.h"
#include <esp_arduino_version.h>

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#define USE_LEDC_V3 1
#else
#define USE_LEDC_V3 0
#endif

// ------------------------------------------------
// MODULE-LEVEL STATE
// Stores the currently commanded motor speeds
// ------------------------------------------------
static int s_leftSpeed  = 0;  // Current left speed
static int s_rightSpeed = 0;  // Current right speed

// ------------------------------------------------
// HELPER: Apply speed to a single motor (4-pin PWM)
// ------------------------------------------------
// pin1, pin2 -> input GPIO pins (IN1/IN2 or IN3/IN4)
// ch1, ch2   -> LEDC PWM channels (for Core 2.x)
// speed      -> -255 to 255 (already deadband-filtered)
// ------------------------------------------------
static void applyMotor(uint8_t pin1, uint8_t pin2,
                       uint8_t ch1, uint8_t ch2, int speed)
{
    if (speed > 0) {
        // Forward: PWM on pin1, LOW on pin2
#if USE_LEDC_V3
        ledcWrite(pin1, (uint32_t)speed);
        ledcWrite(pin2, 0);
#else
        ledcWrite(ch1, (uint32_t)speed);
        ledcWrite(ch2, 0);
#endif
    } else if (speed < 0) {
        // Reverse: LOW on pin1, PWM on pin2
#if USE_LEDC_V3
        ledcWrite(pin1, 0);
        ledcWrite(pin2, (uint32_t)(-speed));
#else
        ledcWrite(ch1, 0);
        ledcWrite(ch2, (uint32_t)(-speed));
#endif
    } else {
        // Coast stop: Both LOW
#if USE_LEDC_V3
        ledcWrite(pin1, 0);
        ledcWrite(pin2, 0);
#else
        ledcWrite(ch1, 0);
        ledcWrite(ch2, 0);
#endif
    }
}

// ------------------------------------------------
// HELPER: Clamp speed to safe limits
// ------------------------------------------------
static int filterSpeed(int raw)
{
    if (raw >  MOTOR_MAX_SPEED) raw =  MOTOR_MAX_SPEED;
    if (raw < -MOTOR_MAX_SPEED) raw = -MOTOR_MAX_SPEED;
    return raw;
}

// =====================================================
// PUBLIC API IMPLEMENTATIONS
// =====================================================

// ------------------------------------------------
// motors_init - Hardware initialisation
// ------------------------------------------------
void motors_init()
{
#if USE_LEDC_V3
    ledcAttach(PIN_MOTOR_L_IN1, PWM_FREQ_MOTOR, PWM_RESOLUTION);
    ledcAttach(PIN_MOTOR_L_IN2, PWM_FREQ_MOTOR, PWM_RESOLUTION);
    ledcAttach(PIN_MOTOR_R_IN3, PWM_FREQ_MOTOR, PWM_RESOLUTION);
    ledcAttach(PIN_MOTOR_R_IN4, PWM_FREQ_MOTOR, PWM_RESOLUTION);

    ledcWrite(PIN_MOTOR_L_IN1, 0);
    ledcWrite(PIN_MOTOR_L_IN2, 0);
    ledcWrite(PIN_MOTOR_R_IN3, 0);
    ledcWrite(PIN_MOTOR_R_IN4, 0);
#else
    ledcSetup(PWM_CH_MOTOR_L_IN1, PWM_FREQ_MOTOR, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_L_IN1, PWM_CH_MOTOR_L_IN1);
    ledcWrite(PWM_CH_MOTOR_L_IN1, 0);

    ledcSetup(PWM_CH_MOTOR_L_IN2, PWM_FREQ_MOTOR, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_L_IN2, PWM_CH_MOTOR_L_IN2);
    ledcWrite(PWM_CH_MOTOR_L_IN2, 0);

    ledcSetup(PWM_CH_MOTOR_R_IN3, PWM_FREQ_MOTOR, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_R_IN3, PWM_CH_MOTOR_R_IN3);
    ledcWrite(PWM_CH_MOTOR_R_IN3, 0);

    ledcSetup(PWM_CH_MOTOR_R_IN4, PWM_FREQ_MOTOR, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_R_IN4, PWM_CH_MOTOR_R_IN4);
    ledcWrite(PWM_CH_MOTOR_R_IN4, 0);
#endif

    s_leftSpeed  = 0;
    s_rightSpeed = 0;
}

// ------------------------------------------------
// motors_set - Set both motor speeds
// ------------------------------------------------
void motors_set(int leftSpeed, int rightSpeed)
{
    s_leftSpeed  = filterSpeed(leftSpeed);
    s_rightSpeed = filterSpeed(rightSpeed);

    applyMotor(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2,
               PWM_CH_MOTOR_L_IN1, PWM_CH_MOTOR_L_IN2, s_leftSpeed);

    applyMotor(PIN_MOTOR_R_IN3, PIN_MOTOR_R_IN4,
               PWM_CH_MOTOR_R_IN3, PWM_CH_MOTOR_R_IN4, s_rightSpeed);
}

// ------------------------------------------------
// motors_stop - Coast stop (let motors spin down)
// ------------------------------------------------
void motors_stop()
{
#if USE_LEDC_V3
    ledcWrite(PIN_MOTOR_L_IN1, 0);
    ledcWrite(PIN_MOTOR_L_IN2, 0);
    ledcWrite(PIN_MOTOR_R_IN3, 0);
    ledcWrite(PIN_MOTOR_R_IN4, 0);
#else
    ledcWrite(PWM_CH_MOTOR_L_IN1, 0);
    ledcWrite(PWM_CH_MOTOR_L_IN2, 0);
    ledcWrite(PWM_CH_MOTOR_R_IN3, 0);
    ledcWrite(PWM_CH_MOTOR_R_IN4, 0);
#endif

    s_leftSpeed  = 0;
    s_rightSpeed = 0;
}

// ------------------------------------------------
// motors_brake - Active brake (short motor windings)
// ------------------------------------------------
void motors_brake()
{
#if USE_LEDC_V3
    ledcWrite(PIN_MOTOR_L_IN1, 255);
    ledcWrite(PIN_MOTOR_L_IN2, 255);
    ledcWrite(PIN_MOTOR_R_IN3, 255);
    ledcWrite(PIN_MOTOR_R_IN4, 255);
#else
    ledcWrite(PWM_CH_MOTOR_L_IN1, 255);
    ledcWrite(PWM_CH_MOTOR_L_IN2, 255);
    ledcWrite(PWM_CH_MOTOR_R_IN3, 255);
    ledcWrite(PWM_CH_MOTOR_R_IN4, 255);
#endif

    s_leftSpeed  = 0;
    s_rightSpeed = 0;
}

// ------------------------------------------------
// motors_get_left / motors_get_right - Read back state
// ------------------------------------------------
int motors_get_left()
{
    return s_leftSpeed;
}

int motors_get_right()
{
    return s_rightSpeed;
}
