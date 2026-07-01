# Hover-Craft
Autonomous Hover Craft Project --> https://youtu.be/6yk7UZK2xvw 

Overview

This project implements the low-level control firmware for an autonomous, fan-propelled vehicle built around the ATmega328P microcontroller. The vehicle drives itself in a straight line using IMU-based yaw correction, detects obstacles with dual ultrasonic sensors, and performs autonomous turning maneuvers to navigate around them — all without relying on the Arduino core libraries, using direct AVR register access for timing, PWM, interrupts, I2C, and UART.

How It Works

The system runs as a simple finite-state machine with three primary states:

IDLE — brief startup delay before the vehicle begins moving.
FORWARD — the vehicle drives forward while continuously reading gyroscope data from an onboard IMU to detect and correct for yaw drift, keeping the vehicle tracking straight. A front-facing ultrasonic sensor monitors for obstacles.
TURN — triggered when an obstacle is detected within a threshold distance. A side-facing ultrasonic sensor checks for clearance, and the steering servo sweeps to steer around the obstacle before returning to forward mode.


Key Hardware & Techniques

Steering: A servo motor provides proportional steering, with servo position dynamically adjusted based on live yaw readings.
Inertial sensing: An MPU6050 IMU (accelerometer + gyroscope) communicates over I2C (TWI) to provide real-time angular rate data used for drift correction and turn resets.
Obstacle detection: Two ultrasonic distance sensors (front and side) use external interrupts (INT0/INT1) to precisely time echo pulses for distance calculation, avoiding blocking delay loops.
(Only two were used due to competitions constraints) 
Propulsion: Two fans (base lift + thrust), provide forward propulsion.
Timing: A custom Timer0-based microsecond/millisecond counter replaces the standard Arduino millis(), enabling precise, interrupt-safe timekeeping for sensor timeouts and mode transitions.
Telemetry: A minimal from-scratch UART driver streams debug and sensor data to a serial monitor for tuning and diagnostics.

Tuning Parameters
Several constants are exposed at the top of the firmware for on-track calibration, including:

YAW_TRIM_GAIN — proportional gain for yaw-based steering correction
TURN_DURATION_MS — how long the vehicle stays in turn mode
SERVO_LEFT / SERVO_CENTER / SERVO_RIGHT — servo PWM endpoints
Obstacle distance thresholds

