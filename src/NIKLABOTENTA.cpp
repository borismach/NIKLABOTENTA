/**
 * @file NIKLABOTENTA.cpp
 * @brief Implementation of NIKLABOTENTA robot library
 * @author Boris Mach
 * @version 0.3.0
 */

#include "NIKLABOTENTA.h"

// Library version
#define NIKLABOTENTA_VERSION "0.3.0"

// Default robot configuration (can be customized)
#define DEFAULT_WHEELBASE_CM 15.0f        // 15cm between wheels
#define DEFAULT_WHEEL_DIAMETER_MM 65.0f   // 65mm wheel diameter

// ============================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================

NIKLABOTENTA::NIKLABOTENTA() :
    _initialized(false),
    _debug_enabled(false),
    _simulation_mode(true),  // Default to simulation mode
    _drive_system(nullptr),
    _imu(nullptr),
    _wheelbase_cm(DEFAULT_WHEELBASE_CM),
    _wheel_diameter_mm(DEFAULT_WHEEL_DIAMETER_MM),
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

    // Create differential drive system
    _drive_system = new DifferentialDrive(_wheelbase_cm,
                                         _wheel_diameter_mm,
                                         _simulation_mode);

    // Create IMU interface (default I2C address 0x28)
    _imu = new BNO055_Interface(55, 0x28, _simulation_mode);
}

NIKLABOTENTA::~NIKLABOTENTA() {
    // Cleanup
    stop();

    if (_drive_system) {
        delete _drive_system;
        _drive_system = nullptr;
    }

    if (_imu) {
        delete _imu;
        _imu = nullptr;
    }
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

    // Initialize differential drive system
    // Default pin configuration (can be customized for your hardware)
    // Left motor: pins 2 (fwd), 3 (bwd)
    // Right motor: pins 4 (fwd), 5 (bwd)
    // Left encoder: pins 6 (A), 7 (B)
    // Right encoder: pins 8 (A), 9 (B)
    // Encoder resolution: 1000 ticks/rev (typical for hobby motor encoders)

    if (_drive_system) {
        bool drive_init = _drive_system->begin(
            2, 3,    // Left motor forward/backward
            4, 5,    // Right motor forward/backward
            6, 7,    // Left encoder A/B
            8, 9,    // Right encoder A/B
            1000     // Encoder ticks per revolution
        );

        if (!drive_init) {
            _debug_print("ERROR: Drive system initialization failed");
            return false;
        }
    }

    // Initialize BNO055 IMU
    if (_imu) {
        _debug_print("Initializing BNO055 IMU...");
        bool imu_init = _imu->begin();

        if (!imu_init && !_simulation_mode) {
            _debug_print("WARNING: BNO055 initialization failed");
            // Continue anyway - IMU is optional
        } else {
            if (_simulation_mode) {
                _debug_print("BNO055 initialized (simulation mode)");
            } else {
                _debug_print("BNO055 initialized successfully");
            }
        }
    }

    // In simulation mode, set calibration
    // In real hardware mode, additional components to initialize:
    // - Nicla Vision camera
    // - Servo controllers
    // - LED controllers

    if (_simulation_mode) {
        _debug_print("Running in SIMULATION mode");
        _imu_calibration = 3;  // Pretend fully calibrated in simulation
    } else {
        _debug_print("Initializing hardware...");
        // TODO: Initialize real hardware in future phases
        // - Configure camera
        // - Setup servo PWM
    }

    _initialized = true;
    _debug_print("Initialization complete");

    return true;
}

// ============================================
// MOVEMENT API
// ============================================

void NIKLABOTENTA::drive(float speed, float rotation) {
    if (!_initialized || !_drive_system) {
        _debug_print("ERROR: Not initialized. Call begin() first.");
        return;
    }

    if (_debug_enabled) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Drive: speed=%.2f cm/s, rot=%.2f deg/s",
                 speed, rotation);
        _debug_print(msg);
    }

    // Convert rotation from deg/s to rad/s
    float rotation_rad_s = rotation * DEG_TO_RAD;

    // Use differential drive system
    _drive_system->drive(speed, rotation_rad_s);

    // Update internal state for compatibility
    _left_speed = _drive_system->get_left_velocity();
    _right_speed = _drive_system->get_right_velocity();

    // Update drive system (important for PID and odometry)
    _drive_system->update();
}

void NIKLABOTENTA::move(float distance, bool blocking) {
    if (!_initialized || !_drive_system) {
        _debug_print("ERROR: Not initialized. Call begin() first.");
        return;
    }

    if (_debug_enabled) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Move: %.2f cm %s",
                 distance, blocking ? "(blocking)" : "(non-blocking)");
        _debug_print(msg);
    }

    // Use differential drive system
    // Default speed: 10 cm/s
    _drive_system->move_distance(distance, 10.0f, blocking);

    // Update pose from drive system
    Pose pose = _drive_system->get_pose();
    _current_x = pose.x;
    _current_y = pose.y;
    _current_heading = pose.theta * RAD_TO_DEG;  // Convert to degrees
    _orientation = _current_heading;
    _yaw = _current_heading;

    // Update velocities
    _left_speed = _drive_system->get_left_velocity();
    _right_speed = _drive_system->get_right_velocity();
}

void NIKLABOTENTA::rotate(float degrees, bool blocking) {
    if (!_initialized || !_drive_system) {
        _debug_print("ERROR: Not initialized. Call begin() first.");
        return;
    }

    if (_debug_enabled) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Rotate: %.2f deg %s",
                 degrees, blocking ? "(blocking)" : "(non-blocking)");
        _debug_print(msg);
    }

    // Convert degrees to radians
    float angle_rad = degrees * DEG_TO_RAD;

    // Use differential drive system
    // Default angular speed: PI/2 rad/s (90 deg/s)
    _drive_system->rotate_angle(angle_rad, PI / 2.0f, blocking);

    // Update pose from drive system
    Pose pose = _drive_system->get_pose();
    _current_x = pose.x;
    _current_y = pose.y;
    _current_heading = pose.theta * RAD_TO_DEG;  // Convert to degrees

    // Normalize to 0-360
    while (_current_heading >= 360.0f) _current_heading -= 360.0f;
    while (_current_heading < 0.0f) _current_heading += 360.0f;

    // Update orientation (same as heading)
    _orientation = _current_heading;
    _yaw = _current_heading;

    // Update velocities
    _left_speed = _drive_system->get_left_velocity();
    _right_speed = _drive_system->get_right_velocity();
}

void NIKLABOTENTA::brake() {
    if (_debug_enabled) {
        _debug_print("Brake");
    }

    if (_drive_system) {
        _drive_system->stop(true);  // Active braking
    }

    _left_speed = 0.0f;
    _right_speed = 0.0f;
}

void NIKLABOTENTA::stop() {
    if (_debug_enabled) {
        _debug_print("Stop");
    }

    if (_drive_system) {
        _drive_system->stop(false);  // Coast to stop
    }

    _left_speed = 0.0f;
    _right_speed = 0.0f;
}

void NIKLABOTENTA::get_speed(float& left, float& right) {
    if (_drive_system) {
        left = _drive_system->get_left_velocity();
        right = _drive_system->get_right_velocity();
    } else {
        left = _left_speed;
        right = _right_speed;
    }
}

// ============================================
// IMU API
// ============================================

float NIKLABOTENTA::get_orientation() {
    if (_imu) {
        // Update IMU readings
        _imu->update();

        // Get heading in degrees (0-360)
        _orientation = _imu->get_heading_degrees();
        _yaw = _orientation;  // Update yaw as well

        return _orientation;
    }

    // Fallback to odometry-based heading
    return _current_heading;
}

float NIKLABOTENTA::get_roll() {
    if (_imu) {
        _imu->update();

        // Get Euler angles in degrees
        float roll_deg, pitch_deg, yaw_deg;
        _imu->get_euler_degrees(roll_deg, pitch_deg, yaw_deg);

        _roll = roll_deg;
        return _roll;
    }

    return 0.0f;  // No roll without IMU
}

float NIKLABOTENTA::get_pitch() {
    if (_imu) {
        _imu->update();

        // Get Euler angles in degrees
        float roll_deg, pitch_deg, yaw_deg;
        _imu->get_euler_degrees(roll_deg, pitch_deg, yaw_deg);

        _pitch = pitch_deg;
        return _pitch;
    }

    return 0.0f;  // No pitch without IMU
}

float NIKLABOTENTA::get_yaw() {
    return get_orientation();  // Yaw is same as orientation
}

void NIKLABOTENTA::get_imu(float& ax, float& ay, float& az,
                           float& gx, float& gy, float& gz) {
    if (_imu) {
        _imu->update();

        // Get acceleration (m/s²)
        Vector3 accel = _imu->get_acceleration();
        ax = accel.x;
        ay = accel.y;
        az = accel.z;

        // Get gyroscope (rad/s)
        Vector3 gyro = _imu->get_gyroscope();
        gx = gyro.x;
        gy = gyro.y;
        gz = gyro.z;

        return;
    }

    // Fallback: simulated values
    ax = 0.0f;
    ay = 0.0f;
    az = 9.81f;  // Gravity
    gx = 0.0f;
    gy = 0.0f;
    gz = 0.0f;
}

void NIKLABOTENTA::get_quaternion(float& w, float& x, float& y, float& z) {
    if (_imu) {
        _imu->update();

        // Get quaternion from BNO055 sensor fusion
        Quaternion q = _imu->get_quaternion();
        w = q.w;
        x = q.x;
        y = q.y;
        z = q.z;

        return;
    }

    // Fallback: identity quaternion
    w = 1.0f;
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
}

void NIKLABOTENTA::get_magnetometer(float& mx, float& my, float& mz) {
    if (_imu) {
        _imu->update();

        // Get magnetometer data (µT)
        Vector3 mag = _imu->get_magnetometer();
        mx = mag.x;
        my = mag.y;
        mz = mag.z;

        return;
    }

    // Fallback: simulated magnetic field
    mx = 30.0f;  // µT (approximate Earth's field)
    my = 0.0f;
    mz = 40.0f;
}

bool NIKLABOTENTA::calibrate_imu() {
    if (!_initialized || !_imu) {
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

    // In real mode, BNO055 calibrates automatically
    // Just wait for full calibration
    _debug_print("Waiting for BNO055 calibration...");
    _debug_print("Move the sensor in figure-8 pattern for magnetometer calibration");

    unsigned long start_time = millis();
    unsigned long timeout = 60000;  // 60 second timeout

    while (millis() - start_time < timeout) {
        _imu->update();

        if (_imu->is_fully_calibrated()) {
            _debug_print("BNO055 fully calibrated!");
            _imu_calibration = 3;
            return true;
        }

        // Print calibration status periodically
        if ((millis() - start_time) % 2000 == 0) {
            CalibrationStatus status = _imu->get_calibration_status();
            char msg[64];
            snprintf(msg, sizeof(msg), "Cal: Sys=%d Gyro=%d Accel=%d Mag=%d",
                     status.system, status.gyro, status.accel, status.mag);
            _debug_print(msg);
        }

        delay(100);
    }

    _debug_print("WARNING: Calibration timeout");
    return false;
}

uint8_t NIKLABOTENTA::get_imu_calibration_status() {
    if (_imu) {
        _imu_calibration = _imu->get_calibration_level();
    }

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
