/**
 * @file BNO055_Interface.cpp
 * @brief Implementation of BNO055 IMU interface
 * @author Boris Mach
 * @version 0.3.0
 */

#include "BNO055_Interface.h"
#include <math.h>

// ============================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================

BNO055_Interface::BNO055_Interface(int32_t sensor_id,
                                   uint8_t address,
                                   bool simulation_mode) :
    _sensor_id(sensor_id),
    _address(address),
    _simulation_mode(simulation_mode),
    _initialized(false)
#ifndef SIMULATION_ONLY
    , _bno(nullptr)
#endif
{
    // Initialize simulated state
    _sim_euler.roll = 0.0f;
    _sim_euler.pitch = 0.0f;
    _sim_euler.yaw = 0.0f;

    _sim_quat.w = 1.0f;
    _sim_quat.x = 0.0f;
    _sim_quat.y = 0.0f;
    _sim_quat.z = 0.0f;

    _sim_accel.x = 0.0f;
    _sim_accel.y = 0.0f;
    _sim_accel.z = 9.81f;  // Gravity

    _sim_gyro.x = 0.0f;
    _sim_gyro.y = 0.0f;
    _sim_gyro.z = 0.0f;

    _sim_mag.x = 30.0f;  // Approximate Earth's field (µT)
    _sim_mag.y = 0.0f;
    _sim_mag.z = 40.0f;

    _sim_calibration.system = 3;
    _sim_calibration.gyro = 3;
    _sim_calibration.accel = 3;
    _sim_calibration.mag = 3;

    // Initialize cached values
    _quaternion = _sim_quat;
    _euler = _sim_euler;
    _acceleration = _sim_accel;
    _gyroscope = _sim_gyro;
    _magnetometer = _sim_mag;
    _calibration = _sim_calibration;
}

BNO055_Interface::~BNO055_Interface() {
#ifndef SIMULATION_ONLY
    if (_bno) {
        delete _bno;
        _bno = nullptr;
    }
#endif
}

// ============================================
// INITIALIZATION
// ============================================

bool BNO055_Interface::begin() {
    if (_initialized) {
        return true;
    }

    if (_simulation_mode) {
        // Simulation mode - always succeeds
        _initialized = true;
        return true;
    }

#ifndef SIMULATION_ONLY
    // Hardware mode - initialize BNO055
    _bno = new Adafruit_BNO055(_sensor_id, _address);

    if (!_bno->begin()) {
        // Failed to initialize
        delete _bno;
        _bno = nullptr;
        return false;
    }

    // Set to NDOF mode (9DOF fusion)
    _bno->setMode(OPERATION_MODE_NDOF);

    // Give sensor time to stabilize
    delay(100);

    _initialized = true;
    return true;
#else
    // If compiled without Adafruit library, can't use hardware
    return false;
#endif
}

// ============================================
// UPDATE
// ============================================

void BNO055_Interface::update() {
    if (!_initialized) {
        return;
    }

    if (_simulation_mode) {
        _update_simulated_data();
        return;
    }

#ifndef SIMULATION_ONLY
    if (_bno) {
        // Read quaternion
        imu::Quaternion q = _bno->getQuat();
        _quaternion.w = q.w();
        _quaternion.x = q.x();
        _quaternion.y = q.y();
        _quaternion.z = q.z();

        // Read Euler angles (in degrees from BNO055, convert to radians)
        imu::Vector<3> euler = _bno->getVector(Adafruit_BNO055::VECTOR_EULER);
        _euler.yaw = euler.x() * DEG_TO_RAD;
        _euler.pitch = euler.y() * DEG_TO_RAD;
        _euler.roll = euler.z() * DEG_TO_RAD;

        // Read accelerometer
        imu::Vector<3> accel = _bno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        _acceleration.x = accel.x();
        _acceleration.y = accel.y();
        _acceleration.z = accel.z();

        // Read gyroscope (convert from deg/s to rad/s)
        imu::Vector<3> gyro = _bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        _gyroscope.x = gyro.x() * DEG_TO_RAD;
        _gyroscope.y = gyro.y() * DEG_TO_RAD;
        _gyroscope.z = gyro.z() * DEG_TO_RAD;

        // Read magnetometer
        imu::Vector<3> mag = _bno->getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        _magnetometer.x = mag.x();
        _magnetometer.y = mag.y();
        _magnetometer.z = mag.z();

        // Read calibration status
        uint8_t sys, gyro, accel, mag;
        _bno->getCalibration(&sys, &gyro, &accel, &mag);
        _calibration.system = sys;
        _calibration.gyro = gyro;
        _calibration.accel = accel;
        _calibration.mag = mag;
    }
#endif
}

// ============================================
// ORIENTATION
// ============================================

Quaternion BNO055_Interface::get_quaternion() {
    return _quaternion;
}

EulerAngles BNO055_Interface::get_euler_angles() {
    return _euler;
}

void BNO055_Interface::get_euler_degrees(float& roll, float& pitch, float& yaw) {
    roll = _euler.roll * RAD_TO_DEG;
    pitch = _euler.pitch * RAD_TO_DEG;
    yaw = _euler.yaw * RAD_TO_DEG;

    // Normalize yaw to 0-360
    while (yaw < 0.0f) yaw += 360.0f;
    while (yaw >= 360.0f) yaw -= 360.0f;
}

float BNO055_Interface::get_heading() {
    return _euler.yaw;
}

float BNO055_Interface::get_heading_degrees() {
    float heading = _euler.yaw * RAD_TO_DEG;

    // Normalize to 0-360
    while (heading < 0.0f) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;

    return heading;
}

// ============================================
// RAW SENSOR DATA
// ============================================

Vector3 BNO055_Interface::get_acceleration() {
    return _acceleration;
}

Vector3 BNO055_Interface::get_gyroscope() {
    return _gyroscope;
}

Vector3 BNO055_Interface::get_magnetometer() {
    return _magnetometer;
}

Vector3 BNO055_Interface::get_linear_acceleration() {
#ifndef SIMULATION_ONLY
    if (_bno && !_simulation_mode) {
        imu::Vector<3> linear = _bno->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        Vector3 result;
        result.x = linear.x();
        result.y = linear.y();
        result.z = linear.z();
        return result;
    }
#endif

    // In simulation, return zero (no movement)
    Vector3 result = {0.0f, 0.0f, 0.0f};
    return result;
}

Vector3 BNO055_Interface::get_gravity() {
#ifndef SIMULATION_ONLY
    if (_bno && !_simulation_mode) {
        imu::Vector<3> grav = _bno->getVector(Adafruit_BNO055::VECTOR_GRAVITY);
        Vector3 result;
        result.x = grav.x();
        result.y = grav.y();
        result.z = grav.z();
        return result;
    }
#endif

    // In simulation, return standard gravity
    Vector3 result = {0.0f, 0.0f, 9.81f};
    return result;
}

// ============================================
// CALIBRATION
// ============================================

CalibrationStatus BNO055_Interface::get_calibration_status() {
    return _calibration;
}

bool BNO055_Interface::is_fully_calibrated() {
    return (_calibration.system == 3) &&
           (_calibration.gyro == 3) &&
           (_calibration.accel == 3) &&
           (_calibration.mag == 3);
}

uint8_t BNO055_Interface::get_calibration_level() {
    return _calibration.system;
}

bool BNO055_Interface::save_calibration() {
#ifndef SIMULATION_ONLY
    if (_bno && !_simulation_mode && is_fully_calibrated()) {
        // Get calibration data
        adafruit_bno055_offsets_t offsets;
        if (_bno->getSensorOffsets(offsets)) {
            // Save to EEPROM (implementation depends on platform)
            // For now, just return success
            return true;
        }
    }
#endif

    return _simulation_mode;  // Always succeed in simulation
}

bool BNO055_Interface::load_calibration() {
#ifndef SIMULATION_ONLY
    if (_bno && !_simulation_mode) {
        // Load from EEPROM and apply
        // Implementation depends on platform
        return false;
    }
#endif

    return _simulation_mode;
}

void BNO055_Interface::reset_calibration() {
#ifndef SIMULATION_ONLY
    if (_bno && !_simulation_mode) {
        // Reset to config mode and back to force recalibration
        _bno->setMode(OPERATION_MODE_CONFIG);
        delay(25);
        _bno->setMode(OPERATION_MODE_NDOF);
    }
#endif

    if (_simulation_mode) {
        // Reset simulation calibration
        _sim_calibration.system = 0;
        _sim_calibration.gyro = 0;
        _sim_calibration.accel = 0;
        _sim_calibration.mag = 0;
        _calibration = _sim_calibration;
    }
}

// ============================================
// TEMPERATURE
// ============================================

int8_t BNO055_Interface::get_temperature() {
#ifndef SIMULATION_ONLY
    if (_bno && !_simulation_mode) {
        return _bno->getTemp();
    }
#endif

    return 25;  // Simulate 25°C
}

// ============================================
// CONFIGURATION
// ============================================

void BNO055_Interface::set_mode(uint8_t mode) {
#ifndef SIMULATION_ONLY
    if (_bno && !_simulation_mode) {
        _bno->setMode(mode);
        delay(25);  // Mode change requires delay
    }
#endif
}

bool BNO055_Interface::is_initialized() const {
    return _initialized;
}

bool BNO055_Interface::is_simulation_mode() const {
    return _simulation_mode;
}

void BNO055_Interface::set_simulated_orientation(float roll, float pitch, float yaw) {
    if (_simulation_mode) {
        _sim_euler.roll = roll;
        _sim_euler.pitch = pitch;
        _sim_euler.yaw = yaw;

        // Convert to quaternion
        _euler_to_quaternion(_sim_euler, _sim_quat);

        // Update cached values
        _euler = _sim_euler;
        _quaternion = _sim_quat;
    }
}

// ============================================
// PRIVATE METHODS
// ============================================

void BNO055_Interface::_update_simulated_data() {
    // In simulation mode, data is static unless explicitly set
    // Copy simulated data to cached values
    _quaternion = _sim_quat;
    _euler = _sim_euler;
    _acceleration = _sim_accel;
    _gyroscope = _sim_gyro;
    _magnetometer = _sim_mag;
    _calibration = _sim_calibration;

    // Simulate gradual calibration if not calibrated
    if (!is_fully_calibrated()) {
        // Slowly increase calibration levels
        if (_sim_calibration.gyro < 3) _sim_calibration.gyro++;
        if (_sim_calibration.accel < 3) _sim_calibration.accel++;
        if (_sim_calibration.mag < 3) _sim_calibration.mag++;
        if (_sim_calibration.system < 3) _sim_calibration.system++;
    }
}

void BNO055_Interface::_euler_to_quaternion(const EulerAngles& euler, Quaternion& quat) {
    // Convert Euler angles to quaternion
    // Using ZYX rotation order (yaw, pitch, roll)
    float cy = cos(euler.yaw * 0.5f);
    float sy = sin(euler.yaw * 0.5f);
    float cp = cos(euler.pitch * 0.5f);
    float sp = sin(euler.pitch * 0.5f);
    float cr = cos(euler.roll * 0.5f);
    float sr = sin(euler.roll * 0.5f);

    quat.w = cr * cp * cy + sr * sp * sy;
    quat.x = sr * cp * cy - cr * sp * sy;
    quat.y = cr * sp * cy + sr * cp * sy;
    quat.z = cr * cp * sy - sr * sp * cy;
}
