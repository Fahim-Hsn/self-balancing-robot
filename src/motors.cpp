/*
 * =====================================================
 * File: motors.cpp
 * Purpose: L298N Mini H-Bridge motor driver implementation
 *          using ESP32 LEDC hardware PWM
 * =====================================================
 * Compatible with both ESP32 Arduino Core 2.x and 3.x.
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
// HELPER: Apply speed to a single motor
// ------------------------------------------------
// pin_in1, pin_in2 -> direction control pins
// pwm_pin          -> PWM GPIO pin number
// pwm_channel      -> LEDC channel number (for Core 2.x)
// speed            -> -255 to 255 (already deadband-filtered)
// ------------------------------------------------
static void applyMotor(uint8_t pin_in1, uint8_t pin_in2,
                       uint8_t pwm_pin, uint8_t pwm_channel, int speed)
{
    if (speed > 0) {
        // Forward
        digitalWrite(pin_in1, HIGH);
        digitalWrite(pin_in2, LOW);
#if USE_LEDC_V3
        ledcWrite(pwm_pin, (uint32_t)speed);
#else
        ledcWrite(pwm_channel, (uint32_t)speed);
#endif
    } else if (speed < 0) {
        // Reverse
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, HIGH);
#if USE_LEDC_V3
        ledcWrite(pwm_pin, (uint32_t)(-speed));
#else
        ledcWrite(pwm_channel, (uint32_t)(-speed));
#endif
    } else {
        // Coast stop
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, LOW);
#if USE_LEDC_V3
        ledcWrite(pwm_pin, 0);
#else
        ledcWrite(pwm_channel, 0);
#endif
    }
}

// ------------------------------------------------
// HELPER: Clamp and apply deadband to raw speed
// ------------------------------------------------
static int filterSpeed(int raw)
{
    if (raw >  MOTOR_MAX_SPEED) raw =  MOTOR_MAX_SPEED;
    if (raw < -MOTOR_MAX_SPEED) raw = -MOTOR_MAX_SPEED;

    if (raw > 0 && raw < MOTOR_MIN_SPEED)  raw = 0;
    if (raw < 0 && raw > -MOTOR_MIN_SPEED) raw = 0;

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
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN3, OUTPUT);
    pinMode(PIN_MOTOR_R_IN4, OUTPUT);

    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);
    digitalWrite(PIN_MOTOR_R_IN3, LOW);
    digitalWrite(PIN_MOTOR_R_IN4, LOW);

#if USE_LEDC_V3
    ledcAttach(PIN_MOTOR_L_PWM, PWM_FREQ_MOTOR, PWM_RESOLUTION);
    ledcWrite(PIN_MOTOR_L_PWM, 0);

    ledcAttach(PIN_MOTOR_R_PWM, PWM_FREQ_MOTOR, PWM_RESOLUTION);
    ledcWrite(PIN_MOTOR_R_PWM, 0);
#else
    ledcSetup(PWM_CH_MOTOR_L, PWM_FREQ_MOTOR, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_L_PWM, PWM_CH_MOTOR_L);
    ledcWrite(PWM_CH_MOTOR_L, 0);

    ledcSetup(PWM_CH_MOTOR_R, PWM_FREQ_MOTOR, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_R_PWM, PWM_CH_MOTOR_R);
    ledcWrite(PWM_CH_MOTOR_R, 0);
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
               PIN_MOTOR_L_PWM, PWM_CH_MOTOR_L, s_leftSpeed);

    applyMotor(PIN_MOTOR_R_IN3, PIN_MOTOR_R_IN4,
               PIN_MOTOR_R_PWM, PWM_CH_MOTOR_R, s_rightSpeed);
}

// ------------------------------------------------
// motors_stop - Coast stop (let motors spin down)
// ------------------------------------------------
void motors_stop()
{
    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);
    digitalWrite(PIN_MOTOR_R_IN3, LOW);
    digitalWrite(PIN_MOTOR_R_IN4, LOW);

#if USE_LEDC_V3
    ledcWrite(PIN_MOTOR_L_PWM, 0);
    ledcWrite(PIN_MOTOR_R_PWM, 0);
#else
    ledcWrite(PWM_CH_MOTOR_L, 0);
    ledcWrite(PWM_CH_MOTOR_R, 0);
#endif

    s_leftSpeed  = 0;
    s_rightSpeed = 0;
}

// ------------------------------------------------
// motors_brake - Active brake (short motor windings)
// ------------------------------------------------
void motors_brake()
{
    digitalWrite(PIN_MOTOR_L_IN1, HIGH);
    digitalWrite(PIN_MOTOR_L_IN2, HIGH);
    digitalWrite(PIN_MOTOR_R_IN3, HIGH);
    digitalWrite(PIN_MOTOR_R_IN4, HIGH);

#if USE_LEDC_V3
    ledcWrite(PIN_MOTOR_L_PWM, 255);
    ledcWrite(PIN_MOTOR_R_PWM, 255);
#else
    ledcWrite(PWM_CH_MOTOR_L, 255);
    ledcWrite(PWM_CH_MOTOR_R, 255);
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
