/**
 * @file BNO055_Interface.h
 * @brief Wrapper for Adafruit BNO055 9-axis IMU sensor
 * @author Boris Mach
 * @version 0.3.0
 *
 * This class provides a simplified interface to the BNO055 sensor with:
 * - 9-axis sensor fusion (accelerometer + gyroscope + magnetometer)
 * - Quaternion orientation output (no gimbal lock)
 * - Euler angles (roll, pitch, yaw)
 * - Automatic calibration management
 * - Both raw sensor data and fusion output
 * - Simulation mode for testing without hardware
 */

#ifndef BNO055_INTERFACE_H
#define BNO055_INTERFACE_H

#include <Arduino.h>

// Forward declaration for Adafruit library (only needed if hardware present)
#ifndef SIMULATION_ONLY
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#endif

/**
 * @brief Calibration status structure
 */
struct CalibrationStatus {
    uint8_t system;      // 0-3 (3 = fully calibrated)
    uint8_t gyro;        // 0-3
    uint8_t accel;       // 0-3
    uint8_t mag;         // 0-3
};

/**
 * @brief Quaternion structure (for orientation without gimbal lock)
 */
struct Quaternion {
    float w;  // Real part
    float x;  // i component
    float y;  // j component
    float z;  // k component
};

/**
 * @brief Euler angles structure (in radians)
 */
struct EulerAngles {
    float roll;   // Rotation around X-axis (-π to π)
    float pitch;  // Rotation around Y-axis (-π/2 to π/2)
    float yaw;    // Rotation around Z-axis (0 to 2π)
};

/**
 * @brief 3D vector structure
 */
struct Vector3 {
    float x;
    float y;
    float z;
};

/**
 * @brief BNO055 IMU interface class
 *
 * Provides simplified access to BNO055 9-axis IMU sensor.
 * Supports both hardware mode (requires Adafruit BNO055 library)
 * and simulation mode for testing.
 */
class BNO055_Interface {
public:
    /**
     * @brief Constructor
     * @param sensor_id Unique ID for this sensor (default: 55)
     * @param address I2C address (default: 0x28, alt: 0x29)
     * @param simulation_mode If true, run without hardware
     */
    BNO055_Interface(int32_t sensor_id = 55,
                     uint8_t address = 0x28,
                     bool simulation_mode = true);

    /**
     * @brief Destructor
     */
    ~BNO055_Interface();

    /**
     * @brief Initialize the BNO055 sensor
     * @return true if successful
     */
    bool begin();

    /**
     * @brief Update sensor readings (call in loop)
     * Updates all cached sensor values
     */
    void update();

    // ============================================
    // ORIENTATION (FUSION OUTPUT)
    // ============================================

    /**
     * @brief Get orientation as quaternion
     * @return Quaternion structure (w, x, y, z)
     */
    Quaternion get_quaternion();

    /**
     * @brief Get orientation as Euler angles
     * @return EulerAngles structure (roll, pitch, yaw in radians)
     */
    EulerAngles get_euler_angles();

    /**
     * @brief Get orientation as Euler angles in degrees
     * @param roll Roll angle in degrees
     * @param pitch Pitch angle in degrees
     * @param yaw Yaw/heading angle in degrees (0-360)
     */
    void get_euler_degrees(float& roll, float& pitch, float& yaw);

    /**
     * @brief Get heading/yaw only (compass direction)
     * @return Heading in radians (0 to 2π)
     */
    float get_heading();

    /**
     * @brief Get heading in degrees
     * @return Heading in degrees (0-360)
     */
    float get_heading_degrees();

    // ============================================
    // RAW SENSOR DATA
    // ============================================

    /**
     * @brief Get accelerometer data (m/s²)
     * @return 3D acceleration vector
     */
    Vector3 get_acceleration();

    /**
     * @brief Get gyroscope data (rad/s)
     * @return 3D angular velocity vector
     */
    Vector3 get_gyroscope();

    /**
     * @brief Get magnetometer data (µT)
     * @return 3D magnetic field vector
     */
    Vector3 get_magnetometer();

    /**
     * @brief Get linear acceleration (gravity removed, m/s²)
     * @return 3D linear acceleration vector
     */
    Vector3 get_linear_acceleration();

    /**
     * @brief Get gravity vector (m/s²)
     * @return 3D gravity vector
     */
    Vector3 get_gravity();

    // ============================================
    // CALIBRATION
    // ============================================

    /**
     * @brief Get calibration status
     * @return CalibrationStatus structure
     */
    CalibrationStatus get_calibration_status();

    /**
     * @brief Check if sensor is fully calibrated
     * @return true if all components calibrated (level 3)
     */
    bool is_fully_calibrated();

    /**
     * @brief Get overall calibration level (0-3)
     * @return System calibration status
     */
    uint8_t get_calibration_level();

    /**
     * @brief Save calibration data to EEPROM
     * @return true if successful
     */
    bool save_calibration();

    /**
     * @brief Load calibration data from EEPROM
     * @return true if successful
     */
    bool load_calibration();

    /**
     * @brief Reset calibration (start fresh)
     */
    void reset_calibration();

    // ============================================
    // TEMPERATURE
    // ============================================

    /**
     * @brief Get sensor temperature
     * @return Temperature in Celsius
     */
    int8_t get_temperature();

    // ============================================
    // CONFIGURATION
    // ============================================

    /**
     * @brief Set operation mode
     * @param mode Operation mode (default: NDOF - 9DOF fusion)
     */
    void set_mode(uint8_t mode);

    /**
     * @brief Check if sensor is initialized
     * @return true if initialized
     */
    bool is_initialized() const;

    /**
     * @brief Check if in simulation mode
     * @return true if simulation mode
     */
    bool is_simulation_mode() const;

    /**
     * @brief Set simulated orientation (for testing)
     * @param roll Roll angle in radians
     * @param pitch Pitch angle in radians
     * @param yaw Yaw angle in radians
     */
    void set_simulated_orientation(float roll, float pitch, float yaw);

    // Operation modes (from Adafruit BNO055 library)
    static const uint8_t OPERATION_MODE_CONFIG = 0x00;
    static const uint8_t OPERATION_MODE_ACCONLY = 0x01;
    static const uint8_t OPERATION_MODE_MAGONLY = 0x02;
    static const uint8_t OPERATION_MODE_GYRONLY = 0x03;
    static const uint8_t OPERATION_MODE_ACCMAG = 0x04;
    static const uint8_t OPERATION_MODE_ACCGYRO = 0x05;
    static const uint8_t OPERATION_MODE_MAGGYRO = 0x06;
    static const uint8_t OPERATION_MODE_AMG = 0x07;
    static const uint8_t OPERATION_MODE_IMUPLUS = 0x08;
    static const uint8_t OPERATION_MODE_COMPASS = 0x09;
    static const uint8_t OPERATION_MODE_M4G = 0x0A;
    static const uint8_t OPERATION_MODE_NDOF_FMC_OFF = 0x0B;
    static const uint8_t OPERATION_MODE_NDOF = 0x0C;  // Default: 9DOF fusion

private:
    // Configuration
    int32_t _sensor_id;
    uint8_t _address;
    bool _simulation_mode;
    bool _initialized;

    // Simulated state (for testing)
    EulerAngles _sim_euler;
    Quaternion _sim_quat;
    Vector3 _sim_accel;
    Vector3 _sim_gyro;
    Vector3 _sim_mag;
    CalibrationStatus _sim_calibration;

    // Cached sensor values
    Quaternion _quaternion;
    EulerAngles _euler;
    Vector3 _acceleration;
    Vector3 _gyroscope;
    Vector3 _magnetometer;
    CalibrationStatus _calibration;

#ifndef SIMULATION_ONLY
    // Adafruit BNO055 instance (only if library available)
    Adafruit_BNO055* _bno;
#endif

    // Internal methods
    void _update_simulated_data();
    void _euler_to_quaternion(const EulerAngles& euler, Quaternion& quat);
};

#endif // BNO055_INTERFACE_H
