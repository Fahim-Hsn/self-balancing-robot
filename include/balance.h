#pragma once

/*
 * =====================================================================
 * FILE: include/balance.h
 * PURPOSE: MPU-6050 raw I2C reading, complementary filter pitch
 *          estimation, and PID controller for self-balancing robot.
 * =====================================================================
 * Hardware: MPU-6050 on I2C bus (SDA=21, SCL=22), shared with OLED.
 * Wire.begin() must be called BEFORE balance_init().
 * =====================================================================
 */

#include <Arduino.h>

// -----------------------------------------------------------------------
// PUBLIC API - Balance module interface
// -----------------------------------------------------------------------

/**
 * balance_init()
 * Configures the MPU-6050 registers over I2C (wake-up, DLPF, ranges).
 * Call once in setup(), after Wire.begin().
 */
void balance_init();

/**
 * balance_calibrate()
 * Blocking calibration: reads 1000 samples, averages gyro bias and
 * accelerometer Z offset. Shows progress via Serial.
 *
 * @return true when calibration completes successfully.
 */
bool balance_calibrate();

/**
 * balance_update()
 * Must be called EVERY loop iteration.
 * Reads raw MPU-6050 data and updates the pitch angle via
 * complementary filter. Rate-limited to max 100 Hz (min 2 ms between
 * updates). Internally tracks dt using micros().
 */
void balance_update();

/**
 * balance_compute(float forwardBias)
 * Runs the PID controller and returns motor output.
 *
 * @param forwardBias  Segway lean bias in degrees.
 *                     Positive = lean forward = move forward.
 *                     Range: -SEGWAY_MAX_TILT to +SEGWAY_MAX_TILT.
 * @return Motor output clamped to [PID_OUTPUT_MIN, PID_OUTPUT_MAX]
 *         (-220 to 220). Zero if robot is fallen.
 */
int balance_compute(float forwardBias);

/**
 * balance_get_angle()
 * @return Latest computed pitch angle in degrees.
 */
float balance_get_angle();

/**
 * balance_get_gyro_rate()
 * @return Latest gyro pitch rate in degrees per second.
 */
float balance_get_gyro_rate();

/**
 * balance_is_fallen()
 * @return true if |angle| > BALANCE_FALLEN_ANGLE.
 */
bool balance_is_fallen();

/**
 * balance_is_tilting()
 * @return true if |angle| > BALANCE_WARN_ANGLE (warning threshold).
 */
bool balance_is_tilting();

/**
 * balance_reset_pid()
 * Resets PID integral and derivative state.
 * Useful when switching modes to prevent integral windup carry-over.
 */
void balance_reset_pid();
