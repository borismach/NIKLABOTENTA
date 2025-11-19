/**
 * @file NIKLABOTENTA.cpp
 * @brief Implementation of NIKLABOTENTA robot library
 * @author Boris Mach
 * @version 0.1.0
 */

#include "NIKLABOTENTA.h"

// Library version
#define NIKLABOTENTA_VERSION "0.1.0"

// ============================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================

NIKLABOTENTA::NIKLABOTENTA() :
    _initialized(false),
    _debug_enabled(false),
    _simulation_mode(true),  // Default to simulation mode
    _left_speed(0.0f),
    _right_speed(0.0f),
    _current_x(0.0f),
    _current_y(0.0f),
    _current_heading(0.0f),
    _led_r(0),
    _led_g(0),
    _led_b(0),
    _line_detected(false),
    _line_position(0.0f),
    _orientation(0.0f),
    _roll(0.0f),
    _pitch(0.0f),
    _yaw(0.0f),
    _imu_calibration(0)
{
    // Initialize servo positions to center (90 degrees)
    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
        _servo_positions[i] = 90.0f;
    }
}

NIKLABOTENTA::~NIKLABOTENTA() {
    // Cleanup if needed
    stop();
}

// ============================================
// INITIALIZATION
// ============================================

bool NIKLABOTENTA::begin() {
    if (_initialized) {
        _debug_print("Already initialized");
        return true;
    }

    _debug_print("Initializing NIKLABOTENTA...");

    // In simulation mode, just set the flag
    // In real hardware mode, initialize actual components:
    // - Motor controllers
    // - BNO055 IMU
    // - Nicla Vision camera
    // - Servo controllers
    // - LED controllers

    if (_simulation_mode) {
        _debug_print("Running in SIMULATION mode");
        _imu_calibration = 3;  // Pretend fully calibrated in simulation
    } else {
        _debug_print("Initializing hardware...");
        // TODO: Initialize real hardware in future phases
        // - Setup I2C for BNO055
        // - Initialize motor PWM
        // - Configure camera
    }

    _initialized = true;
    _debug_print("Initialization complete");

    return true;
}

// ============================================
// MOVEMENT API
// ============================================

void NIKLABOTENTA::drive(float speed, float rotation) {
    if (!_initialized) {
        _debug_print("ERROR: Not initialized. Call begin() first.");
        return;
    }

    // Differential drive kinematics
    // Left wheel = speed - rotation
    // Right wheel = speed + rotation
    _left_speed = speed - rotation;
    _right_speed = speed + rotation;

    if (_debug_enabled) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Drive: speed=%.2f, rot=%.2f -> L=%.2f, R=%.2f",
                 speed, rotation, _left_speed, _right_speed);
        _debug_print(msg);
    }

    // In simulation mode, just update internal state
    // In real mode, send commands to motor controllers
}

void NIKLABOTENTA::move(float distance, bool blocking) {
    if (!_initialized) {
        _debug_print("ERROR: Not initialized. Call begin() first.");
        return;
    }

    if (_debug_enabled) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Move: %.2f cm %s",
                 distance, blocking ? "(blocking)" : "(non-blocking)");
        _debug_print(msg);
    }

    // Simulate movement by updating position
    float rad = _current_heading * DEG_TO_RAD;
    _current_x += distance * cos(rad);
    _current_y += distance * sin(rad);

    // In real mode: calculate encoder ticks, set motor speeds, wait if blocking
    if (blocking && _simulation_mode) {
        // Simulate time to complete movement (assume 10 cm/s)
        unsigned long delay_ms = abs(distance) * 100;  // ms = cm * 100
        delay(delay_ms);
    }

    // Stop after movement
    stop();
}

void NIKLABOTENTA::rotate(float degrees, bool blocking) {
    if (!_initialized) {
        _debug_print("ERROR: Not initialized. Call begin() first.");
        return;
    }

    if (_debug_enabled) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Rotate: %.2f deg %s",
                 degrees, blocking ? "(blocking)" : "(non-blocking)");
        _debug_print(msg);
    }

    // Update heading
    _current_heading += degrees;

    // Normalize to 0-360
    while (_current_heading >= 360.0f) _current_heading -= 360.0f;
    while (_current_heading < 0.0f) _current_heading += 360.0f;

    // Update orientation (same as heading in simple case)
    _orientation = _current_heading;
    _yaw = _current_heading;

    // In real mode: calculate rotation speed, wait if blocking
    if (blocking && _simulation_mode) {
        // Simulate time to complete rotation (assume 90 deg/s)
        unsigned long delay_ms = abs(degrees) * 11;  // ms = deg * 11
        delay(delay_ms);
    }

    stop();
}

void NIKLABOTENTA::brake() {
    if (_debug_enabled) {
        _debug_print("Brake");
    }

    _left_speed = 0.0f;
    _right_speed = 0.0f;

    // In real mode: apply active braking to motors
}

void NIKLABOTENTA::stop() {
    if (_debug_enabled) {
        _debug_print("Stop");
    }

    _left_speed = 0.0f;
    _right_speed = 0.0f;

    // In real mode: coast motors to stop
}

void NIKLABOTENTA::get_speed(float& left, float& right) {
    left = _left_speed;
    right = _right_speed;
}

// ============================================
// IMU API
// ============================================

float NIKLABOTENTA::get_orientation() {
    // In simulation, return current heading
    // In real mode, read from BNO055
    return _orientation;
}

float NIKLABOTENTA::get_roll() {
    // In simulation, return simulated roll
    // In real mode, read from BNO055
    return _roll;
}

float NIKLABOTENTA::get_pitch() {
    // In simulation, return simulated pitch
    // In real mode, read from BNO055
    return _pitch;
}

float NIKLABOTENTA::get_yaw() {
    // In simulation, return current heading
    // In real mode, read from BNO055
    return _yaw;
}

void NIKLABOTENTA::get_imu(float& ax, float& ay, float& az,
                           float& gx, float& gy, float& gz) {
    // In simulation, return simulated values
    ax = 0.0f;
    ay = 0.0f;
    az = 9.81f;  // Gravity
    gx = 0.0f;
    gy = 0.0f;
    gz = 0.0f;

    // In real mode, read from BNO055 accelerometer and gyroscope
}

void NIKLABOTENTA::get_quaternion(float& w, float& x, float& y, float& z) {
    // In simulation, return identity quaternion
    w = 1.0f;
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;

    // In real mode, read quaternion from BNO055 (fusion output)
}

void NIKLABOTENTA::get_magnetometer(float& mx, float& my, float& mz) {
    // In simulation, return simulated magnetic field
    mx = 30.0f;  // µT (approximate Earth's field)
    my = 0.0f;
    mz = 40.0f;

    // In real mode, read from BNO055 magnetometer
}

bool NIKLABOTENTA::calibrate_imu() {
    if (!_initialized) {
        return false;
    }

    _debug_print("Calibrating IMU...");

    if (_simulation_mode) {
        // Simulate calibration delay
        delay(1000);
        _imu_calibration = 3;  // Fully calibrated
        _debug_print("IMU calibration complete (simulated)");
        return true;
    }

    // In real mode, perform BNO055 calibration procedure
    // This involves reading calibration status and waiting
    return false;
}

uint8_t NIKLABOTENTA::get_imu_calibration_status() {
    return _imu_calibration;
}

// ============================================
// SERVO CONTROL API
// ============================================

void NIKLABOTENTA::set_servo_position(uint8_t servo_id, float degrees) {
    if (servo_id >= MAX_SERVOS) {
        _debug_print("ERROR: Invalid servo ID");
        return;
    }

    // Clamp to valid range
    if (degrees < 0.0f) degrees = 0.0f;
    if (degrees > 180.0f) degrees = 180.0f;

    _servo_positions[servo_id] = degrees;

    if (_debug_enabled) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Servo %d: %.2f deg", servo_id, degrees);
        _debug_print(msg);
    }

    // In real mode, send PWM command to servo
}

float NIKLABOTENTA::get_servo_position(uint8_t servo_id) {
    if (servo_id >= MAX_SERVOS) {
        return 0.0f;
    }
    return _servo_positions[servo_id];
}

// ============================================
// VISION API
// ============================================

bool NIKLABOTENTA::detect_line() {
    // In simulation, return simulated state
    // In real mode, process camera image for line detection
    return _line_detected;
}

float NIKLABOTENTA::get_line_position() {
    // Return cached line position
    return _line_position;
}

bool NIKLABOTENTA::detect_color(uint8_t& r, uint8_t& g, uint8_t& b) {
    // In simulation, return simulated color
    r = 128;
    g = 128;
    b = 128;

    // In real mode, capture image and analyze color
    return true;
}

bool NIKLABOTENTA::detect_object() {
    // In simulation, return false
    // In real mode, use camera for object detection
    return false;
}

float NIKLABOTENTA::get_distance() {
    // In simulation, return simulated distance
    // In real mode, read from Nicla Vision proximity sensor
    return 50.0f;  // cm
}

// ============================================
// LED CONTROL API
// ============================================

void NIKLABOTENTA::set_led_color(uint8_t r, uint8_t g, uint8_t b) {
    _led_r = r;
    _led_g = g;
    _led_b = b;

    if (_debug_enabled) {
        char msg[64];
        snprintf(msg, sizeof(msg), "LED: R=%d G=%d B=%d", r, g, b);
        _debug_print(msg);
    }

    // In real mode, control RGB LED hardware
}

void NIKLABOTENTA::led_off() {
    set_led_color(0, 0, 0);
}

// ============================================
// POWER MANAGEMENT API
// ============================================

float NIKLABOTENTA::get_battery_voltage() {
    // In simulation, return simulated voltage
    // In real mode, read from ADC connected to battery
    return 7.4f;  // Typical LiPo 2S voltage
}

bool NIKLABOTENTA::is_charging() {
    // In simulation, return false
    // In real mode, read charging status pin
    return false;
}

// ============================================
// UTILITY API
// ============================================

void NIKLABOTENTA::set_debug(bool enable) {
    _debug_enabled = enable;
    if (enable) {
        Serial.println("Debug output enabled");
    }
}

const char* NIKLABOTENTA::get_version() {
    return NIKLABOTENTA_VERSION;
}

bool NIKLABOTENTA::is_simulation_mode() {
    return _simulation_mode;
}

// ============================================
// PRIVATE METHODS
// ============================================

void NIKLABOTENTA::_update_odometry() {
    // Future: update position based on encoder readings
    // For now, odometry is updated directly in move() and rotate()
}

void NIKLABOTENTA::_debug_print(const char* message) {
    if (_debug_enabled) {
        Serial.print("[NIKLABOTENTA] ");
        Serial.println(message);
    }
}
