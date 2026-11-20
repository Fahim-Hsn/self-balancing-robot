#pragma once

/*
 * =====================================================
 * File: buzzer.h
 * Purpose: Non-blocking buzzer API for RoboBot
 *          using FreeRTOS task on Core 0
 * =====================================================
 * All melodies and tones run in a FreeRTOS task on Core 0
 * so the main control loop is never blocked.
 * =====================================================
 */

#include <Arduino.h>
#include "config.h"

// ------------------------------------------------
// BUZZER SOUND COMMANDS
// Sent to the FreeRTOS task via a queue
// ------------------------------------------------
enum BuzzerCmd {
    BUZZ_STARTUP,           // Startup melody
    BUZZ_MODE_0,            // Mode 0 - balance
    BUZZ_MODE_1,            // Mode 1 - drive
    BUZZ_BALANCE_LOST,      // Balance lost alert
    BUZZ_WIFI_OK,           // WiFi connected
    BUZZ_OBSTACLE_AVOIDED   // Obstacle avoided
};

// ------------------------------------------------
// PUBLIC API
// ------------------------------------------------

/**
 * @brief Initialise buzzer LEDC channel and start FreeRTOS sound task.
 */
void buzzer_init();

/**
 * @brief Queue the startup melody (4-note ascending arpeggio).
 */
void buzzer_startup_melody();

/**
 * @brief Queue the melody for a mode change.
 * @param newMode The mode just activated.
 */
void buzzer_mode_changed(RobotMode newMode);

/**
 * @brief Queue the balance-lost alert (3x low buzz).
 */
void buzzer_balance_lost();

/**
 * @brief Queue the WiFi-connected chime.
 */
void buzzer_wifi_connected();

/**
 * @brief Queue the obstacle-avoided victory sound.
 */
void buzzer_obstacle_avoided();

/**
 * @brief Enable or disable all buzzer output.
 * @param en true = enabled, false = silent.
 */
void buzzer_set_enabled(bool en);

/**
 * @brief Returns true if buzzer is currently enabled.
 */
bool buzzer_is_enabled();

/**
 * @brief Call from main loop - handles proximity-based obstacle beep
 *        timing using millis() (non-blocking).
 */
void buzzer_update();

/**
 * @brief Inform buzzer module of current obstacle distance.
 * @param distCm Distance in centimetres. Pass -1 if no valid reading.
 */
void buzzer_obstacle_set_distance(float distCm);
