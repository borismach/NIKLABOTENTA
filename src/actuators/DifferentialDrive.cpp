/**
 * @file DifferentialDrive.cpp
 * @brief Implementation of differential drive kinematics
 * @author Boris Mach
 * @version 0.2.0
 */

#include "DifferentialDrive.h"

// ============================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================

DifferentialDrive::DifferentialDrive(float wheelbase_cm,
                                     float wheel_diameter_mm,
                                     bool simulation_mode) :
    _wheelbase_cm(wheelbase_cm),
    _wheel_diameter_mm(wheel_diameter_mm),
    _left_motor(nullptr),
    _right_motor(nullptr),
    _left_encoder(nullptr),
    _right_encoder(nullptr),
    _left_pid(nullptr),
    _right_pid(nullptr),
    _left_velocity(0.0f),
    _right_velocity(0.0f),
    _last_update_time(0),
    _simulation_mode(simulation_mode),
    _initialized(false),
    _pid_enabled(true),
    _move_state(MOVE_IDLE),
    _move_target(0.0f),
    _move_start_value(0.0f)
{
    // Calculate wheel radius
    _wheel_radius_cm = wheel_diameter_mm / 20.0f;  // Convert to cm radius

    // Initialize pose at origin
    _pose.x = 0.0f;
    _pose.y = 0.0f;
    _pose.theta = 0.0f;
}

DifferentialDrive::~DifferentialDrive() {
    // Cleanup dynamically allocated components
    if (_left_motor) delete _left_motor;
    if (_right_motor) delete _right_motor;
    if (_left_encoder) delete _left_encoder;
    if (_right_encoder) delete _right_encoder;
    if (_left_pid) delete _left_pid;
    if (_right_pid) delete _right_pid;
}

// ============================================
// INITIALIZATION
// ============================================

bool DifferentialDrive::begin(uint8_t left_motor_fwd, uint8_t left_motor_bwd,
                               uint8_t right_motor_fwd, uint8_t right_motor_bwd,
                               uint8_t left_enc_a, uint8_t left_enc_b,
                               uint8_t right_enc_a, uint8_t right_enc_b,
                               uint16_t encoder_ticks_per_rev) {
    if (_initialized) {
        return true;
    }

    // Create motor controllers
    _left_motor = new MotorController(left_motor_fwd, left_motor_bwd, _simulation_mode);
    _right_motor = new MotorController(right_motor_fwd, right_motor_bwd, _simulation_mode);

    // Create encoders
    _left_encoder = new Encoder(left_enc_a, left_enc_b,
                                encoder_ticks_per_rev,
                                _wheel_diameter_mm,
                                _simulation_mode);
    _right_encoder = new Encoder(right_enc_a, right_enc_b,
                                 encoder_ticks_per_rev,
                                 _wheel_diameter_mm,
                                 _simulation_mode);

    // Create PID controllers (default gains, can be tuned later)
    _left_pid = new PIDController(1.0f, 0.1f, 0.05f);
    _right_pid = new PIDController(1.0f, 0.1f, 0.05f);

    // Set PID output limits to motor speed range
    _left_pid->set_output_limits(-100.0f, 100.0f);
    _right_pid->set_output_limits(-100.0f, 100.0f);

    // Initialize components
    if (!_left_motor->begin() || !_right_motor->begin()) {
        return false;
    }

    if (!_left_encoder->begin() || !_right_encoder->begin()) {
        return false;
    }

    _last_update_time = millis();
    _initialized = true;

    return true;
}

// ============================================
// UPDATE
// ============================================

void DifferentialDrive::update() {
    if (!_initialized) {
        return;
    }

    unsigned long current_time = millis();
    unsigned long dt_ms = current_time - _last_update_time;

    if (dt_ms >= UPDATE_INTERVAL_MS) {
        float dt = dt_ms / 1000.0f;  // Convert to seconds

        // Update encoders
        _left_encoder->update();
        _right_encoder->update();

        // Update odometry
        _update_odometry(dt);

        // Update PID controllers if enabled
        if (_pid_enabled) {
            _update_pid_control(dt);
        }

        _last_update_time = current_time;
    }
}

// ============================================
// DRIVE CONTROL
// ============================================

void DifferentialDrive::drive(float linear_velocity, float angular_velocity) {
    if (!_initialized) {
        return;
    }

    // Convert robot velocities to wheel velocities
    float left_vel, right_vel;
    _inverse_kinematics(linear_velocity, angular_velocity, left_vel, right_vel);

    set_wheel_velocities(left_vel, right_vel);
}

void DifferentialDrive::set_wheel_velocities(float left_velocity, float right_velocity) {
    if (!_initialized) {
        return;
    }

    _left_velocity = left_velocity;
    _right_velocity = right_velocity;

    // If PID is disabled, apply velocities directly as motor speeds
    if (!_pid_enabled) {
        // Convert cm/s to approximate motor speed percentage
        // Assuming max speed of 50 cm/s = 100%
        float left_speed = (left_velocity / 50.0f) * 100.0f;
        float right_speed = (right_velocity / 50.0f) * 100.0f;

        _left_motor->set_speed(left_speed);
        _right_motor->set_speed(right_speed);

        // In simulation, update encoder velocities
        if (_simulation_mode) {
            _left_encoder->set_simulated_velocity(left_velocity);
            _right_encoder->set_simulated_velocity(right_velocity);
        }
    }
    // If PID enabled, the update() function will handle motor control
}

bool DifferentialDrive::move_distance(float distance_cm, float speed_cm_s, bool blocking) {
    if (!_initialized) {
        return false;
    }

    // Reset encoders at start of movement
    if (_move_state == MOVE_IDLE) {
        _left_encoder->reset_distance();
        _right_encoder->reset_distance();
        _move_state = MOVE_DISTANCE;
        _move_target = abs(distance_cm);
        _move_start_value = 0.0f;
    }

    // Determine direction
    float direction = (distance_cm >= 0.0f) ? 1.0f : -1.0f;

    // Drive forward/backward
    drive(direction * speed_cm_s, 0.0f);

    // Check if target reached
    float avg_distance = (_left_encoder->get_distance() + _right_encoder->get_distance()) / 2.0f;

    if (avg_distance >= _move_target) {
        stop();
        _move_state = MOVE_IDLE;
        return true;
    }

    // In blocking mode, wait until complete
    if (blocking) {
        while (avg_distance < _move_target) {
            update();
            avg_distance = (_left_encoder->get_distance() + _right_encoder->get_distance()) / 2.0f;
            delay(10);
        }
        stop();
        _move_state = MOVE_IDLE;
        return true;
    }

    return false;
}

bool DifferentialDrive::rotate_angle(float angle_rad, float speed_rad_s, bool blocking) {
    if (!_initialized) {
        return false;
    }

    // Reset at start
    if (_move_state == MOVE_IDLE) {
        _move_state = MOVE_ROTATE;
        _move_target = abs(angle_rad);
        _move_start_value = _pose.theta;
    }

    // Determine direction
    float direction = (angle_rad >= 0.0f) ? 1.0f : -1.0f;

    // Rotate (angular velocity only)
    drive(0.0f, direction * speed_rad_s);

    // Check if target reached
    float angle_diff = abs(_pose.theta - _move_start_value);

    // Handle angle wrapping
    if (angle_diff > PI) {
        angle_diff = TWO_PI - angle_diff;
    }

    if (angle_diff >= _move_target) {
        stop();
        _move_state = MOVE_IDLE;
        return true;
    }

    // Blocking mode
    if (blocking) {
        while (angle_diff < _move_target) {
            update();
            angle_diff = abs(_pose.theta - _move_start_value);
            if (angle_diff > PI) {
                angle_diff = TWO_PI - angle_diff;
            }
            delay(10);
        }
        stop();
        _move_state = MOVE_IDLE;
        return true;
    }

    return false;
}

void DifferentialDrive::stop(bool use_brake) {
    if (!_initialized) {
        return;
    }

    _left_velocity = 0.0f;
    _right_velocity = 0.0f;

    _left_motor->stop(use_brake);
    _right_motor->stop(use_brake);

    if (_simulation_mode) {
        _left_encoder->set_simulated_velocity(0.0f);
        _right_encoder->set_simulated_velocity(0.0f);
    }

    _move_state = MOVE_IDLE;
}

// ============================================
// POSE / ODOMETRY
// ============================================

Pose DifferentialDrive::get_pose() const {
    return _pose;
}

void DifferentialDrive::reset_pose(float x, float y, float theta) {
    _pose.x = x;
    _pose.y = y;
    _pose.theta = theta;

    if (_left_encoder) _left_encoder->reset_distance();
    if (_right_encoder) _right_encoder->reset_distance();
}

// ============================================
// VELOCITY GETTERS
// ============================================

float DifferentialDrive::get_left_velocity() const {
    if (_left_encoder) {
        return _left_encoder->get_velocity();
    }
    return _left_velocity;
}

float DifferentialDrive::get_right_velocity() const {
    if (_right_encoder) {
        return _right_encoder->get_velocity();
    }
    return _right_velocity;
}

float DifferentialDrive::get_linear_velocity() const {
    float linear, angular;
    _forward_kinematics(get_left_velocity(), get_right_velocity(),
                       linear, angular);
    return linear;
}

float DifferentialDrive::get_angular_velocity() const {
    float linear, angular;
    _forward_kinematics(get_left_velocity(), get_right_velocity(),
                       linear, angular);
    return angular;
}

// ============================================
// CONFIGURATION
// ============================================

void DifferentialDrive::set_pid_gains(float kp, float ki, float kd) {
    if (_left_pid) _left_pid->set_gains(kp, ki, kd);
    if (_right_pid) _right_pid->set_gains(kp, ki, kd);
}

void DifferentialDrive::enable_pid(bool enable) {
    _pid_enabled = enable;

    if (!enable && _left_motor && _right_motor) {
        // Reset PID state when disabled
        if (_left_pid) _left_pid->reset();
        if (_right_pid) _right_pid->reset();
    }
}

bool DifferentialDrive::is_pid_enabled() const {
    return _pid_enabled;
}

void DifferentialDrive::set_motor_inversion(bool left_inverted, bool right_inverted) {
    if (_left_motor) _left_motor->set_inverted(left_inverted);
    if (_right_motor) _right_motor->set_inverted(right_inverted);
}

// ============================================
// PRIVATE METHODS
// ============================================

void DifferentialDrive::_update_odometry(float dt) {
    // Get wheel velocities from encoders
    float v_left = _left_encoder->get_velocity();   // cm/s
    float v_right = _right_encoder->get_velocity(); // cm/s

    // Calculate robot velocities
    float v_linear, v_angular;
    _forward_kinematics(v_left, v_right, v_linear, v_angular);

    // Update pose using Euler integration
    _pose.x += v_linear * cos(_pose.theta) * dt;
    _pose.y += v_linear * sin(_pose.theta) * dt;
    _pose.theta += v_angular * dt;

    // Normalize theta to -PI to PI
    while (_pose.theta > PI) _pose.theta -= TWO_PI;
    while (_pose.theta < -PI) _pose.theta += TWO_PI;
}

void DifferentialDrive::_update_pid_control(float dt) {
    // Get measured velocities
    float v_left_measured = _left_encoder->get_velocity();
    float v_right_measured = _right_encoder->get_velocity();

    // Compute PID outputs
    float left_output = _left_pid->compute(_left_velocity, v_left_measured, dt);
    float right_output = _right_pid->compute(_right_velocity, v_right_measured, dt);

    // Apply to motors
    _left_motor->set_speed(left_output);
    _right_motor->set_speed(right_output);

    // In simulation, update encoder velocities
    if (_simulation_mode) {
        // PID output affects simulated velocity (simplified model)
        _left_encoder->set_simulated_velocity(_left_velocity);
        _right_encoder->set_simulated_velocity(_right_velocity);
    }
}

void DifferentialDrive::_inverse_kinematics(float linear_vel, float angular_vel,
                                            float& left_vel, float& right_vel) {
    // Differential drive inverse kinematics:
    // v_left = v_linear - (angular_vel * wheelbase / 2)
    // v_right = v_linear + (angular_vel * wheelbase / 2)

    left_vel = linear_vel - (angular_vel * _wheelbase_cm / 2.0f);
    right_vel = linear_vel + (angular_vel * _wheelbase_cm / 2.0f);
}

void DifferentialDrive::_forward_kinematics(float left_vel, float right_vel,
                                            float& linear_vel, float& angular_vel) {
    // Differential drive forward kinematics:
    // v_linear = (v_left + v_right) / 2
    // angular_vel = (v_right - v_left) / wheelbase

    linear_vel = (left_vel + right_vel) / 2.0f;
    angular_vel = (right_vel - left_vel) / _wheelbase_cm;
}
