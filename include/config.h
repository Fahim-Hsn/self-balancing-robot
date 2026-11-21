#pragma once

/*
 * =====================================================
 * RoboBot - Shared Configuration Header
 * Central configuration file for all modules.
 * =====================================================
 * Defines all pin allocations, enumerations, and constants.
 * All C++ source modules include this single header file.
 * =====================================================
 */

// ------------------------------------------------
// PIN DEFINITIONS
// ------------------------------------------------

// I2C Bus - MPU-6050 and OLED share the same I2C bus
#define PIN_SDA     21
#define PIN_SCL     22

// L298N Mini Motor Driver (MX1508 4-pin direct PWM control)
// Left Motor (MOTOR-A)
#define PIN_MOTOR_L_IN1  25   // PWM Left Motor forward
#define PIN_MOTOR_L_IN2  26   // PWM Left Motor reverse

// Right Motor (MOTOR-B)
#define PIN_MOTOR_R_IN3  27   // PWM Right Motor forward
#define PIN_MOTOR_R_IN4  14   // PWM Right Motor reverse

// Ultrasonic Sensor HC-SR04
#define PIN_TRIG    5         // Trigger output
#define PIN_ECHO    18        // Echo input, interrupt-capable

// Buzzer
#define PIN_BUZZER  19        // Active or passive buzzer

// Push Buttons
// GPIO 32 and 33 support internal pull-ups (INPUT_PULLUP) without external resistors.
// Note: GPIO 34 and 35 are input-only and require external 10k pull-up resistors.
#define PIN_BTN_MODE      32  // Button 1: Mode switch (INPUT_PULLUP)
#define PIN_BTN_OBSTACLE  33  // Button 2: Obstacle toggle (INPUT_PULLUP)

// ------------------------------------------------
// ESP32 LEDC PWM CHANNELS
// ------------------------------------------------
#define PWM_CH_MOTOR_L_IN1 0   // Left motor IN1 PWM channel
#define PWM_CH_MOTOR_L_IN2 1   // Left motor IN2 PWM channel
#define PWM_CH_MOTOR_R_IN3 2   // Right motor IN3 PWM channel
#define PWM_CH_MOTOR_R_IN4 3   // Right motor IN4 PWM channel
#define PWM_CH_BUZZER      4   // Buzzer channel
#define PWM_FREQ_MOTOR  20000 // 20 kHz for motors (inaudible, smooth N20 torque)
#define PWM_RESOLUTION   8    // 8-bit resolution: 0 - 255

// ------------------------------------------------
// OPERATING MODES
// ------------------------------------------------
enum RobotMode {
    // Mode 0: Self-balance + Captive Portal Drive (Segway style)
    MODE_BALANCE_SEGWAY = 0,
    // Mode 1: Regular Captive Portal Drive (flat surface)
    MODE_REGULAR_DRIVE  = 1
};

// ------------------------------------------------
// EYE EXPRESSIONS (14 distinct states)
// ------------------------------------------------
enum EyeExpression {
    EYE_NORMAL,      // Normal - idle / stationary
    EYE_HAPPY,       // Happy - well-balanced and moving
    EYE_SURPRISED,   // Surprised - obstacle ahead
    EYE_ANGRY,       // Angry - high tilt / losing balance
    EYE_SLEEPY,      // Sleepy - short idle
    EYE_LOOK_LEFT,   // Look left - steering left
    EYE_LOOK_RIGHT,  // Look right - steering right
    EYE_DIZZY,       // Dizzy - fallen over / extreme tilt
    EYE_COOL,        // Cool - sunglasses style boot sequence
    EYE_SMIRK,       // Smirk - captive portal mode active
    EYE_ZZZ,         // Deep sleep - extended idle
    EYE_HEART,       // Heart - obstacle successfully avoided
    EYE_SWEAT,       // Sweat - heavy drive load
    EYE_SPINNING     // Spinning - WiFi connecting
};

// ------------------------------------------------
// OBSTACLE THRESHOLDS (cm)
// ------------------------------------------------
#define OBSTACLE_WARN_CM   30   // Distance below which warning beep begins
#define OBSTACLE_SLOW_CM   20   // Distance below which motor speed reduces
#define OBSTACLE_STOP_CM   15   // Distance below which vehicle stops/evades

// ------------------------------------------------
// PID CONTROLLER (N20 Low-RPM Motors)
// ------------------------------------------------
/*
 * PID TUNING STRATEGY for N20 Low-RPM motors:
 *
 * Step 1: Set Ki=0, Kd=0. Tune Kp ONLY until robot oscillates slowly.
 * Step 2: If oscillating fast -> lower Kp. If falls without trying -> raise Kp.
 * Step 3: Once Kp gives slow gentle oscillation, add small Kd to damp it.
 * Step 4: Add tiny Ki last to fix steady lean.
 *
 * CURRENT TUNING: Start conservative - P only, no D, no I.
 * Kp=12 gives gentle correction without overshoot on slow N20 motors.
 * Raise Kp by 2 steps at a time if robot falls too slowly / barely reacts.
 */
#define PID_KP            16.0f   // Proportional gain
#define PID_KI             0.0f   // Integral gain (tune last after stable)
#define PID_KD             0.6f   // Derivative damping (now properly in-phase with gyro)
#define PID_TARGET_ANGLE   0.0f   // Balance setpoint (degrees, adjust with dashboard Trim)

#define PID_OUTPUT_MAX    230.0f  // Cap below 255 to prevent saturation oscillation
#define PID_OUTPUT_MIN   -230.0f  // Min motor output
#define PID_INTEGRAL_MAX   20.0f  // Anti-windup clamp

// Segway drive: joystick Y maps to this max tilt bias (degrees)
#define SEGWAY_MAX_TILT    4.0f

// ------------------------------------------------
// BALANCE SAFETY
// ------------------------------------------------
#define BALANCE_FALLEN_ANGLE  35.0f  // Angle cutoff: shut off motors if fallen
#define BALANCE_WARN_ANGLE    18.0f  // Warning threshold for angry eyes

// ------------------------------------------------
// MOTOR LIMITS
// ------------------------------------------------
#define MOTOR_MAX_SPEED   255    // Maximum PWM (0 - 255)
// N20 starting torque: raise if robot barely moves at small angles,
// lower if it vibrates wildly when near upright
#define MOTOR_MIN_SPEED    48    // Starting PWM to overcome N20 gearbox friction

// ------------------------------------------------
// TIMING CONSTANTS (milliseconds)
// ------------------------------------------------
#define BUTTON_DEBOUNCE_MS       50    // Button debounce interval
#define DISPLAY_MODE_SHOW_MS   2500    // Mode info overlay duration
#define IDLE_SLEEPY_MS         8000    // Sleepy expression delay
#define IDLE_ZZZ_MS           20000    // ZZZ expression delay
#define OBSTACLE_TRIGGER_MS      50    // Ultrasonic trigger interval

// ------------------------------------------------
// WiFi / CAPTIVE PORTAL
// ------------------------------------------------
#define WIFI_SSID       "RoboBot"     // Access Point SSID
#define WIFI_PASSWORD   ""            // Open network (no password)

// ------------------------------------------------
// MPU-6050 RAW I2C CONFIGURATION
// ------------------------------------------------
#define MPU6050_ADDR     0x68   // Default I2C address (AD0 to GND)
#define MPU_ACCEL_SCALE  16384.0f  // +/-2g default: 16384 LSB/g
#define MPU_GYRO_SCALE   131.0f    // +/-250 deg/s default: 131 LSB/(deg/s)

// Complementary filter coefficient: 95% gyro, 5% accelerometer
// Lower alpha corrects gyro drift faster - important for low-RPM motor balance
#define COMP_FILTER_ALPHA  0.95f

// ------------------------------------------------
// DISPLAY CONFIGURATION
// ------------------------------------------------
#define DISPLAY_WIDTH   128
#define DISPLAY_HEIGHT   64
