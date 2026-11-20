/*
 * =====================================================
 * File: motors.cpp
 * Purpose: L298N Mini H-Bridge motor driver implementation
 *          using ESP32 LEDC hardware PWM
 * =====================================================
 * Controls speed and direction of two N20 gear motors.
 * =====================================================
 */

#include "motors.h"

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
// pwm_channel      -> LEDC channel number
// speed            -> -255 to 255 (already deadband-filtered)
// ------------------------------------------------
static void applyMotor(uint8_t pin_in1, uint8_t pin_in2,
                       uint8_t pwm_channel, int speed)
{
    if (speed > 0) {
        // Forward
        digitalWrite(pin_in1, HIGH);
        digitalWrite(pin_in2, LOW);
        ledcWrite(pwm_channel, (uint32_t)speed);
    } else if (speed < 0) {
        // Reverse
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, HIGH);
        ledcWrite(pwm_channel, (uint32_t)(-speed));
    } else {
        // Coast stop
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, LOW);
        ledcWrite(pwm_channel, 0);
    }
}

// ------------------------------------------------
// HELPER: Clamp and apply deadband to raw speed
// ------------------------------------------------
static int filterSpeed(int raw)
{
    // Clamp within +/-MOTOR_MAX_SPEED
    if (raw >  MOTOR_MAX_SPEED) raw =  MOTOR_MAX_SPEED;
    if (raw < -MOTOR_MAX_SPEED) raw = -MOTOR_MAX_SPEED;

    // Deadband: if |speed| below minimum threshold, force to 0
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
    // Configure direction pins as OUTPUT
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN3, OUTPUT);
    pinMode(PIN_MOTOR_R_IN4, OUTPUT);

    // Ensure motors start stopped (coast)
    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);
    digitalWrite(PIN_MOTOR_R_IN3, LOW);
    digitalWrite(PIN_MOTOR_R_IN4, LOW);

    // Setup LEDC PWM channels
    // Channel 0: Left motor
    ledcSetup(PWM_CH_MOTOR_L, PWM_FREQ_MOTOR, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_L_PWM, PWM_CH_MOTOR_L);
    ledcWrite(PWM_CH_MOTOR_L, 0);

    // Channel 1: Right motor
    ledcSetup(PWM_CH_MOTOR_R, PWM_FREQ_MOTOR, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_R_PWM, PWM_CH_MOTOR_R);
    ledcWrite(PWM_CH_MOTOR_R, 0);

    // Reset stored speed state
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

    // Drive left motor (IN1, IN2, CH0)
    applyMotor(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2,
               PWM_CH_MOTOR_L, s_leftSpeed);

    // Drive right motor (IN3, IN4, CH1)
    applyMotor(PIN_MOTOR_R_IN3, PIN_MOTOR_R_IN4,
               PWM_CH_MOTOR_R, s_rightSpeed);
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

    ledcWrite(PWM_CH_MOTOR_L, 0);
    ledcWrite(PWM_CH_MOTOR_R, 0);

    s_leftSpeed  = 0;
    s_rightSpeed = 0;
}

// ------------------------------------------------
// motors_brake - Active brake (short motor windings)
// ------------------------------------------------
void motors_brake()
{
    // Both IN pins HIGH -> motor windings shorted to GND
    digitalWrite(PIN_MOTOR_L_IN1, HIGH);
    digitalWrite(PIN_MOTOR_L_IN2, HIGH);
    digitalWrite(PIN_MOTOR_R_IN3, HIGH);
    digitalWrite(PIN_MOTOR_R_IN4, HIGH);

    // Keep PWM at max to maximise braking force
    ledcWrite(PWM_CH_MOTOR_L, 255);
    ledcWrite(PWM_CH_MOTOR_R, 255);

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
