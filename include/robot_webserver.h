#pragma once

/*
 * =====================================================
 * RoboBot - robot_webserver.h
 * Web server module header file.
 * =====================================================
 * ESP32 WiFi Access Point + Captive Portal DNS server.
 * Enables browser-based robot driving and live telemetry.
 * All domains redirect to 192.168.4.1.
 * =====================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include "config.h"

// ------------------------------------------------
// WEB COMMAND STRUCTURE
// ------------------------------------------------
// Holds the latest control command received from the web UI.
// main.cpp reads this struct to drive the motors.
struct WebCommand {
    int  joyX;           // -100 to 100 | Left(-) / Right(+) turn
    int  joyY;           // -100 to 100 | Reverse(-) / Forward(+)
    int  speed;          // 0 to 100    | Speed percentage
    bool buzzerEnabled;  // true = buzzer active
    int  requestedMode;  // -1 = no change, 0 = MODE_BALANCE_SEGWAY, 1 = MODE_REGULAR_DRIVE
    bool invertMotors;   // true = invert motor direction
    bool triggerCalib;   // true = trigger recalibration
    bool updatePid;      // true = new PID settings received
    float pidKp;         // Proportional gain
    float pidKi;         // Integral gain
    float pidKd;         // Derivative gain
    float pidTrim;       // Target balance angle trim (degrees)
    bool updated;        // true = new command received
};

// ------------------------------------------------
// PUBLIC API
// ------------------------------------------------

/**
 * webserver_init()
 * Links the shared WebCommand struct before starting the server.
 *
 * @param cmd Pointer to the global WebCommand in main.cpp
 */
void webserver_init(WebCommand* cmd);

/**
 * webserver_start()
 * Starts the WiFi Access Point, DNS server (captive portal),
 * and the HTTP server. Registers all routes.
 */
void webserver_start();

/**
 * webserver_update()
 * Must be called every loop(). Processes DNS and HTTP requests.
 */
void webserver_update();

/**
 * webserver_is_connected()
 * Returns true if at least one client is connected to the AP.
 */
bool webserver_is_connected();

/**
 * webserver_set_telemetry()
 * Updates the telemetry data returned by GET /status.
 */
void webserver_set_telemetry(float angle, float distance,
                              const char* expression,
                              RobotMode mode,
                              bool obsEnabled, bool buzzEnabled,
                              int motorL, int motorR,
                              float kp, float ki, float kd, float trim);
