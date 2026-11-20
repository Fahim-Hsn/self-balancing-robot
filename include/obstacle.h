#pragma once

/*
 * =====================================================
 * File: obstacle.h
 * Purpose: HC-SR04 ultrasonic obstacle sensor API
 *          with interrupt-based echo measurement
 * =====================================================
 * Non-blocking: trigger fires every 50 ms via millis().
 * Echo pulse width measured by GPIO interrupt on PIN_ECHO.
 * =====================================================
 */

#include <Arduino.h>
#include "config.h"

// ------------------------------------------------
// OBSTACLE LEVEL ENUM
// ------------------------------------------------
enum ObstacleLevel {
    OBS_CLEAR = 0,  // Path clear (> 30 cm)
    OBS_WARN  = 1,  // Warning    (20 - 30 cm)
    OBS_SLOW  = 2,  // Slow down  (15 - 20 cm)
    OBS_STOP  = 3   // Full stop  (< 15 cm)
};

// ------------------------------------------------
// ISR PROTOTYPE
// ------------------------------------------------
void echoISR();

// ------------------------------------------------
// PUBLIC API
// ------------------------------------------------

/**
 * @brief Initialise ultrasonic sensor pins and attach echo interrupt.
 */
void obstacle_init();

/**
 * @brief Call from main loop - fires trigger pulse every 50 ms.
 *        Non-blocking; uses millis() timer internally.
 */
void obstacle_update();

/**
 * @brief Returns the last measured distance in centimetres.
 * @return Distance in cm, or -1.0 if no valid reading yet.
 */
float obstacle_get_distance();

/**
 * @brief Returns the current obstacle threat level.
 * @return ObstacleLevel enum value.
 */
ObstacleLevel obstacle_get_level();

/**
 * @brief Returns whether obstacle detection is enabled.
 */
bool obstacle_is_enabled();

/**
 * @brief Enable or disable obstacle sensing.
 * @param en true = enabled, false = disabled.
 */
void obstacle_set_enabled(bool en);
