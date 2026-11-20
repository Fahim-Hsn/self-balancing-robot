# RoboBot - ESP32 Self-Balancing and Obstacle Detection Car

Dual Operating Modes:
1. Self-Balance + Segway Drive (Inverted pendulum PID control + web joystick driving)
2. Regular Captive Portal Drive (Flat surface RC driving + automatic obstacle evasion)

---

## Project Overview

RoboBot is an advanced dual-wheel robotic vehicle powered by the ESP32 microcontroller. Key features include:

- Self-balancing control via MPU-6050 6-DOF IMU (raw I2C registers), complementary filtering, and closed-loop PID algorithm.
- 1.3-inch I2C OLED display (U8g2 library) rendering 14 animated box-type robot eye expressions that react dynamically to motion, orientation, and obstacles.
- HC-SR04 ultrasonic rangefinder with interrupt-driven, non-blocking timing providing three-stage proximity detection and automated evasion.
- Built-in WiFi Captive Portal Access Point (SSID: RoboBot) serving a dark-themed control dashboard with touch joystick, direction buttons, speed slider, live telemetry, and real-time eye expression mirroring. No external app or internet connection is required.
- Non-blocking sound generator powered by a dedicated FreeRTOS background task pinned to Core 0, keeping the Core 1 control loop completely unhindered.
- Fully modular PlatformIO C++ firmware architecture with clear English code comments.

---

## Hardware Components

| Component | Specification / Notes |
|---|---|
| Microcontroller | ESP32-WROOM-32 Development Board |
| Motors | 2x N20 Micro Metal Gear Motor |
| Motor Driver | L298N Mini Dual H-Bridge Motor Driver Module |
| IMU Sensor | GY-521 MPU-6050 (Mounted flat/horizontal, face up) |
| Distance Sensor | HC-SR04 Ultrasonic Sensor |
| Display | 1.3-inch I2C OLED Display (SH1106 controller, 128x64) |
| Audio Output | 2-Leg Buzzer Module |
| Push Buttons | 2x Tactile Push Button |
| Pull-Up Resistors | 2x 10k-ohm (1/4W) resistors for GPIO 34 and GPIO 35 |
| Power Supply 1 | 5V 3A (ESP32, Sensors, Logic) |
| Power Supply 2 | 5V 3A (L298N Motor Power Vm) |

---

## Complete Wiring Chart

Important: A common ground connection is strictly required between all power supplies and modules. The ground rail of the motor supply and the ESP32 logic ground rail must be connected together.

### ESP32 Pin Allocation Table

| ESP32 Pin | Target Component | Module Pin | Signal / Function | Voltage Level |
|---|---|---|---|---|
| GPIO 21 | OLED (1.3") + MPU-6050 | SDA | Shared I2C Data line | 3.3V Logic |
| GPIO 22 | OLED (1.3") + MPU-6050 | SCL | Shared I2C Clock line | 3.3V Logic |
| GPIO 25 | L298N Mini Driver | IN1 (Motor A) | Left motor forward PWM | 3.3V Logic |
| GPIO 26 | L298N Mini Driver | IN2 (Motor A) | Left motor reverse PWM | 3.3V Logic |
| GPIO 27 | L298N Mini Driver | IN3 (Motor B) | Right motor forward PWM | 3.3V Logic |
| GPIO 14 | L298N Mini Driver | IN4 (Motor B) | Right motor reverse PWM | 3.3V Logic |
| GPIO 5 | HC-SR04 Ultrasonic | TRIG | Sensor trigger pulse output | 3.3V Logic |
| GPIO 18 | HC-SR04 Ultrasonic | ECHO | Sensor echo input (Interrupt-driven) | 3.3V Logic |
| GPIO 19 | 2-Leg Buzzer Module | + / SIG | Audio PWM tone output (LEDC Ch 2) | PWM |
| GPIO 34 | Button 1 (Mode Switch) | Terminal 1 | External 10k pull-up to 3.3V, button to GND | 3.3V Logic |
| GPIO 35 | Button 2 (Obstacle Toggle) | Terminal 1 | External 10k pull-up to 3.3V, button to GND | 3.3V Logic |
| 3.3V | MPU-6050, Pull-Ups | VCC | 3.3V regulated power rail | 3.3V DC |
| 5V / VIN | OLED, HC-SR04, Buzzer | VCC | 5V regulated logic power rail | 5V DC |
| GND | All Modules | GND | Common ground connection rail | 0V Reference |

---

### Dual Power Supply Wiring

To prevent motor electrical noise and current spikes from resetting the ESP32, dual power separation is implemented:

```
[ Supply 1: 5V 3A (Power Bank / Regulator) ]
  +5V  --------------------------> ESP32 VIN, OLED VCC, HC-SR04 VCC, Buzzer VCC
  GND  ------+-------------------> ESP32 GND, Sensors GND
             |
             | (COMMON GROUND TIE)
             |
[ Supply 2: 5V 3A (Motor Battery / Supply) ]
  +5V  --------------------------> L298N Mini Motor VCC (Vm)
  GND  ------+-------------------> L298N Mini GND
```

Motor Output Connections:
- Left N20 Motor  -> L298N Mini OUT1 and OUT2
- Right N20 Motor -> L298N Mini OUT3 and OUT4
- If a motor rotates in the wrong direction, invert the two wires connected to its respective output terminals.

---

### External Pull-Up Wiring for Push Buttons

GPIO 34 and GPIO 35 on the ESP32 are input-only pins without internal pull-up or pull-down resistors. External 10k-ohm pull-up resistors are required:

```
3.3V Rail ---[ 10k Resistor ]---+--- GPIO 34 (Mode Button)
                                |
                                +---[ Push Button 1 ]---> GND Rail

3.3V Rail ---[ 10k Resistor ]---+--- GPIO 35 (Obstacle Button)
                                |
                                +---[ Push Button 2 ]---> GND Rail
```

When unpressed, the input pin reads HIGH (3.3V). When pressed, the input connects directly to GND and reads LOW.

---

## Operating Modes and Button Operation

### Button 1 (GPIO 34): Operating Mode Toggle
- Mode 0: Self-Balance + Segway Drive
  - Maintains upright equilibrium using MPU-6050 pitch data and PID control.
  - Web dashboard joystick adjusts the balance setpoint forward or backward, driving the robot like a personal transporter.
- Mode 1: Regular Drive
  - Disables balance PID for flat 3-wheel or 4-wheel chassis operation.
  - Directly drives motors from web control joystick and speed slider.

### Button 2 (GPIO 35): Obstacle Avoidance Toggle
- Toggles obstacle detection ON and OFF in either operating mode.
- The OLED displays an informational status overlay whenever a button is pressed.

---

## Obstacle Detection Response Logic

| Distance | Vehicle Response | Buzzer Feedback |
|---|---|---|
| Greater than 30 cm | Normal driving permitted | Silent |
| 20 cm to 30 cm | Normal driving, warning active | Slow pulsing beep (600 ms interval) |
| 15 cm to 20 cm | Speed reduced by 50% | Medium pulsing beep (300 ms interval) |
| Less than 15 cm | Balance Mode: Stop driving<br>Drive Mode: Stop, reverse 300 ms, turn 500 ms | Rapid alarm beep (100 ms interval) |

---

## Animated Robot Box Eye Expressions

The 1.3-inch OLED and the web dashboard mirror 14 distinct expressions:

| Expression Name | Trigger Condition | Visual Appearance |
|---|---|---|
| EYE_NORMAL | Stationary / Idle state | Box eye with slow blink animation |
| EYE_HAPPY | Balanced and moving forward | Upward curved smile arc |
| EYE_SURPRISED | Obstacle detected ahead (<30 cm) | Expanded outer box and larger pupil |
| EYE_ANGRY | Excessive tilt angle (>20 deg) | Slanted downward eyebrows |
| EYE_SLEEPY | Inactive for over 8 seconds | Upper eyelid dropped over 55% of eye |
| EYE_ZZZ | Inactive for over 20 seconds | Half-lidded eye with floating Z characters |
| EYE_LOOK_LEFT | Steering left | Pupils shifted toward left edge |
| EYE_LOOK_RIGHT | Steering right | Pupils shifted toward right edge |
| EYE_DIZZY | Fallen over (|tilt| > 40 deg) | Animated rotating X cross pupils |
| EYE_COOL | Startup boot sequence | Full-width sunglasses frame |
| EYE_SMIRK | Captive portal initialized | Asymmetrical right eye smirk |
| EYE_HEART | Obstacle successfully cleared | Heart-shaped pupils |
| EYE_SWEAT | High motor drive load | Animated dropping tear on right side |
| EYE_SPINNING | WiFi establishing | 4 radiating rotating line segments |

---

## WiFi Captive Portal Web Dashboard

1. Power on the robot and wait for calibration to finish.
2. On any smartphone, tablet, or laptop, open WiFi settings and connect to the network named RoboBot (Open, no password).
3. The configuration dashboard opens automatically via captive portal redirection.
4. If the page does not open automatically, open any web browser and navigate to 192.168.4.1.

Dashboard Features:
- Virtual Touch Joystick: 360-degree proportional steering and forward/reverse control.
- D-Pad Buttons: Discrete directional controls (UP, DOWN, LEFT, RIGHT, STOP).
- Speed Slider: Configures overall drive velocity scaling from 0% to 100%.
- Real-Time Eye Canvas: HTML5 canvas rendering mirroring the physical OLED expressions live.
- Live Telemetry: Pitch angle, obstacle clearance, PWM motor output, and active mode badge.
- Buzzer Switch: Toggle sound output directly from the browser.

---

## PID Controller Tuning Guide

PID constants are located in `include/config.h`:

```cpp
#define PID_KP            22.0f   // Proportional gain
#define PID_KI           150.0f   // Integral gain
#define PID_KD             1.2f   // Derivative gain
#define PID_TARGET_ANGLE   0.0f   // Desired equilibrium angle
```

Tuning Procedure:
1. Target Angle: If the robot consistently falls forward, decrease PID_TARGET_ANGLE by 0.5 degrees (e.g. -0.5, -1.0). If it falls backward, increase it.
2. Proportional Gain (KP): Increase KP until the robot reacts promptly to tilts. If high-frequency oscillation occurs, reduce KP.
3. Derivative Gain (KD): Increase KD to damp out oscillations and stabilize sudden movements.
4. Integral Gain (KI): Increase KI slightly to eliminate steady-state offset and prevent slow continuous drifting.

---

## PlatformIO Build and Upload Instructions

1. Install Visual Studio Code and the PlatformIO IDE extension.
2. Open this folder in VS Code (`File -> Open Folder`).
3. Connect the ESP32 board to the computer using a micro-USB data cable.
4. Click the PlatformIO Build button (checkmark icon) on the bottom toolbar.
5. Click the Upload button (right arrow icon) to flash the firmware.
6. Open the Serial Monitor at 115200 baud to view startup diagnostics and calibration logs.

---

## Project Structure

```
self balance car/
|-- platformio.ini         # PlatformIO build configuration and dependencies
|-- min_spiffs.csv         # Custom partition table (1.9MB application space)
|-- README.md              # Project documentation and wiring instructions
|-- include/
|   |-- config.h           # Central pin definitions, PID gains, thresholds
|   |-- balance.h          # MPU-6050 raw I2C driver and PID header
|   |-- buzzer.h           # Non-blocking FreeRTOS buzzer header
|   |-- display.h          # U8g2 OLED and robot eye animation header
|   |-- motors.h           # L298N LEDC hardware PWM motor control header
|   |-- obstacle.h         # HC-SR04 interrupt ultrasonic header
|   `-- robot_webserver.h  # WiFi captive portal and web server header
|-- src/
|   |-- main.cpp           # Main loop, mode handling, state machine
|   |-- balance.cpp        # MPU-6050 complementary filter and PID algorithm
|   |-- buzzer.cpp         # FreeRTOS Core 0 sound task implementation
|   |-- display.cpp        # OLED renderer for 14 robot eye animations
|   |-- motors.cpp         # Dual N20 PWM motor driver implementation
|   |-- obstacle.cpp       # Interrupt-driven ultrasonic pulse measurement
|   `-- webserver.cpp      # Captive portal DNS and embedded HTML dashboard
`-- data/
    `-- index.html         # Standalone web dashboard interface file
```
