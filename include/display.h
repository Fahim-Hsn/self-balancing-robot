#pragma once

/*
 * =====================================================
 * File   : include/display.h
 * Project: RoboBot - ESP32 Self-Balance Car
 * Purpose: OLED Display API - Animated Robot Box Eyes
 * =====================================================
 * Hardware:
 *   - 1.3" OLED, 128x64 pixels
 *   - SH1106 controller (most 1.3" OLEDs)
 *   - SSD1306 controller (most 0.96" OLEDs)
 *   - Shared I2C bus: SDA=21, SCL=22
 *   - Library: U8g2 (full buffer mode for smooth animation)
 *
 * API Overview:
 *   display_init()                   - Initialize display
 *   display_show_startup_animation() - Startup sequence
 *   display_set_expression(expr)     - Set eye expression
 *   display_get_expression()         - Get current expression
 *   display_show_mode_info(...)      - Show mode overlay
 *   display_show_text(...)           - Show text message
 *   display_set_distance(cm)         - Provide sensor data
 *   display_set_angle(deg)           - Provide balance angle
 *   display_update()                 - Call every loop (20 fps)
 * =====================================================
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include "config.h"

// =====================================================
// U8g2 DISPLAY CONSTRUCTOR
// =====================================================
// Uncomment the line matching your OLED controller chip.
// Most 1.3" OLEDs use SH1106 (default).
extern U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2;
// extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2; // For 0.96" OLED

// =====================================================
// PUBLIC API
// =====================================================

/**
 * Initialize the OLED display.
 * Call once in setup() after Wire.begin().
 */
void display_init();

/**
 * Play the startup animation sequence (blocking).
 * Call once after display_init().
 */
void display_show_startup_animation();

/**
 * Set the robot eye expression.
 * @param expr One of the EyeExpression enum values (config.h)
 */
void display_set_expression(EyeExpression expr);

/**
 * Get the current eye expression.
 * @return Current EyeExpression
 */
EyeExpression display_get_expression();

/**
 * Show a rounded-rectangle mode info overlay for DISPLAY_MODE_SHOW_MS ms.
 * @param mode Current robot mode
 * @param obstacleEnabled Whether obstacle avoidance is on
 * @param buzzerEnabled Whether buzzer is on
 */
void display_show_mode_info(RobotMode mode, bool obstacleEnabled = true, bool buzzerEnabled = true);

/**
 * Show a temporary text message on the OLED.
 * @param line1 First line of text
 * @param line2 Second line of text (optional)
 */
void display_show_text(const char* line1, const char* line2 = nullptr);

/**
 * Provide latest obstacle distance for display logic.
 * @param distCm Distance in centimetres (-1 = no reading)
 */
void display_set_distance(float distCm);

/**
 * Provide latest balance angle for display logic.
 * @param angleDeg Angle in degrees
 */
void display_set_angle(float angleDeg);

/**
 * Main display update - call every loop iteration.
 * Throttles to ~20 fps. Handles all blink/animation timers.
 */
void display_update();
