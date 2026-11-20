#pragma once

/*
 * =====================================================
 * File: motors.h
 * Purpose: Motor control API for L298N Mini Dual H-Bridge
 *          with 2x N20 gear motors
 * =====================================================
 * Uses ESP32 LEDC hardware PWM for smooth speed control.
 * =====================================================
 */

#include <Arduino.h>
#include "config.h"

// ------------------------------------------------
// PUBLIC API
// ------------------------------------------------

/**
 * @brief Initialize motor driver pins and LEDC PWM channels.
 */
void motors_init();

/**
 * @brief Set speed for both motors.
 *
 * @param leftSpeed Left motor speed (-255 to 255). Negative = reverse.
 * @param rightSpeed Right motor speed (-255 to 255). Negative = reverse.
 *
 * Values are clamped to +/-MOTOR_MAX_SPEED.
 * If |speed| < MOTOR_MIN_SPEED the motor is stopped (deadband).
 */
void motors_set(int leftSpeed, int rightSpeed);

/**
 * @brief Coast stop - remove drive (both IN pins LOW).
 */
void motors_stop();

/**
 * @brief Active brake - both IN pins HIGH, motor shorted to ground.
 */
void motors_brake();

/**
 * @brief Get current left motor speed command.
 * @return Speed in range -255 to 255.
 */
int motors_get_left();

/**
 * @brief Get current right motor speed command.
 * @return Speed in range -255 to 255.
 */
int motors_get_right();
