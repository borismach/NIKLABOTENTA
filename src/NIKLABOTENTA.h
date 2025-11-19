/**
 * @file NIKLABOTENTA.h
 * @brief Alvik-compatible robot library for Nicla Vision, Portenta H7, and BNO055 IMU
 * @author Boris Mach
 * @version 0.4.0
 *
 * This library provides an Alvik-compatible API for building custom robots using:
 * - Arduino Nicla Vision (camera, proximity sensor)
 * - Arduino Portenta H7 (dual-core processor)
 * - Adafruit BNO055 STEMMA (9-axis IMU with sensor fusion)
 *
 * Features both Alvik-compatible methods and extended capabilities.
 */

#ifndef NIKLABOTENTA_H
#define NIKLABOTENTA_H

#include <Arduino.h>
#include "actuators/DifferentialDrive.h"
#include "sensors/BNO055_Interface.h"
#include "sensors/NiclaVision_Interface.h"

/**
 * @brief Main robot control class
 *
 * This class provides both Alvik-compatible API and extended features
 * for advanced capabilities like computer vision and 9-axis IMU.
 */
class NIKLABOTENTA {
public:
    /**
     * @brief Constructor
     */
    NIKLABOTENTA();

    /**
     * @brief Destructor
     */
    ~NIKLABOTENTA();

    // ============================================
    // INITIALIZATION
    // ============================================

    /**
     * @brief Initialize the robot and all sensors
     * @return true if initialization successful, false otherwise
     */
    bool begin();

    // ============================================
    // MOVEMENT API (Alvik-compatible)
    // ============================================

    /**
     * @brief Drive with specified speed and rotation
     * @param speed Linear speed in cm/s (positive = forward, negative = backward)
     * @param rotation Angular velocity in deg/s (positive = right, negative = left)
     */
    void drive(float speed, float rotation);

    /**
     * @brief Move forward or backward by a specific distance
     * @param distance Distance in centimeters (positive = forward, negative = backward)
     * @param blocking If true, wait until movement is complete
     */
    void move(float distance, bool blocking = true);

    /**
     * @brief Rotate by a specific angle
     * @param degrees Angle in degrees (positive = clockwise, negative = counterclockwise)
     * @param blocking If true, wait until rotation is complete
     */
    void rotate(float degrees, bool blocking = true);

    /**
     * @brief Apply brakes to motors
     */
    void brake();

    /**
     * @brief Stop motors (coast to stop)
     */
    void stop();

    /**
     * @brief Get current speed
     * @param left Reference to store left wheel speed (cm/s)
     * @param right Reference to store right wheel speed (cm/s)
     */
    void get_speed(float& left, float& right);

    // ============================================
    // IMU API (Alvik-compatible + Extended)
    // ============================================

    /**
     * @brief Get heading/orientation (yaw angle)
     * @return Heading in degrees (0-360)
     */
    float get_orientation();

    /**
     * @brief Get roll angle
     * @return Roll in degrees (-180 to 180)
     */
    float get_roll();

    /**
     * @brief Get pitch angle
     * @return Pitch in degrees (-90 to 90)
     */
    float get_pitch();

    /**
     * @brief Get yaw angle
     * @return Yaw in degrees (0-360)
     */
    float get_yaw();

    /**
     * @brief Get raw IMU data (accelerometer + gyroscope)
     * @param ax Acceleration X (m/s²)
     * @param ay Acceleration Y (m/s²)
     * @param az Acceleration Z (m/s²)
     * @param gx Gyroscope X (rad/s)
     * @param gy Gyroscope Y (rad/s)
     * @param gz Gyroscope Z (rad/s)
     */
    void get_imu(float& ax, float& ay, float& az,
                 float& gx, float& gy, float& gz);

    /**
     * @brief Get orientation as quaternion (Extended feature - BNO055)
     * @param w Quaternion W component
     * @param x Quaternion X component
     * @param y Quaternion Y component
     * @param z Quaternion Z component
     */
    void get_quaternion(float& w, float& x, float& y, float& z);

    /**
     * @brief Get magnetometer data (Extended feature - BNO055)
     * @param mx Magnetic field X (µT)
     * @param my Magnetic field Y (µT)
     * @param mz Magnetic field Z (µT)
     */
    void get_magnetometer(float& mx, float& my, float& mz);

    /**
     * @brief Calibrate IMU
     * @return true if calibration successful
     */
    bool calibrate_imu();

    /**
     * @brief Get IMU calibration status
     * @return Calibration level (0-3, 3 = fully calibrated)
     */
    uint8_t get_imu_calibration_status();

    // ============================================
    // SERVO CONTROL API (Alvik-compatible)
    // ============================================

    /**
     * @brief Set servo position
     * @param servo_id Servo identifier (0-based index)
     * @param degrees Angle in degrees (0-180)
     */
    void set_servo_position(uint8_t servo_id, float degrees);

    /**
     * @brief Get current servo position
     * @param servo_id Servo identifier (0-based index)
     * @return Current angle in degrees
     */
    float get_servo_position(uint8_t servo_id);

    // ============================================
    // VISION API (Extended feature - Nicla Vision)
    // ============================================

    /**
     * @brief Detect line using camera
     * @return true if line detected
     */
    bool detect_line();

    /**
     * @brief Get detected line position
     * @return Line position (-1.0 to 1.0, 0 = center, -1 = left, 1 = right)
     */
    float get_line_position();

    /**
     * @brief Detect color using camera
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     * @return true if color detected successfully
     */
    bool detect_color(uint8_t& r, uint8_t& g, uint8_t& b);

    /**
     * @brief Detect object using camera
     * @return true if object detected
     */
    bool detect_object();

    /**
     * @brief Get distance to detected object
     * @return Distance in centimeters (using proximity sensor)
     */
    float get_distance();

    // ============================================
    // LED CONTROL API (Alvik-compatible)
    // ============================================

    /**
     * @brief Set LED color
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     */
    void set_led_color(uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Turn off LED
     */
    void led_off();

    // ============================================
    // POWER MANAGEMENT API (Alvik-compatible)
    // ============================================

    /**
     * @brief Get battery voltage
     * @return Voltage in volts
     */
    float get_battery_voltage();

    /**
     * @brief Check if battery is charging
     * @return true if charging
     */
    bool is_charging();

    // ============================================
    // UTILITY API
    // ============================================

    /**
     * @brief Enable/disable debug output
     * @param enable true to enable debug output to Serial
     */
    void set_debug(bool enable);

    /**
     * @brief Get library version
     * @return Version string (e.g., "0.1.0")
     */
    const char* get_version();

    /**
     * @brief Check if hardware is in simulation mode
     * @return true if running without physical hardware
     */
    bool is_simulation_mode();

private:
    // Internal state
    bool _initialized;
    bool _debug_enabled;
    bool _simulation_mode;

    // Motor control system
    DifferentialDrive* _drive_system;

    // IMU sensor
    BNO055_Interface* _imu;

    // Camera and vision
    NiclaVision_Interface* _camera;

    // Robot configuration
    float _wheelbase_cm;
    float _wheel_diameter_mm;

    // Movement state (for compatibility)
    float _left_speed;
    float _right_speed;
    float _current_x;
    float _current_y;
    float _current_heading;

    // Servo positions
    static const uint8_t MAX_SERVOS = 4;
    float _servo_positions[MAX_SERVOS];

    // LED state
    uint8_t _led_r, _led_g, _led_b;

    // Vision state
    bool _line_detected;
    float _line_position;

    // IMU state
    float _orientation;
    float _roll, _pitch, _yaw;
    uint8_t _imu_calibration;

    // Internal helper methods
    void _update_odometry();
    void _debug_print(const char* message);
};

#endif // NIKLABOTENTA_H
