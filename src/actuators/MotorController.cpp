/**
 * @file MotorController.cpp
 * @brief Implementation of DC Motor controller
 * @author Boris Mach
 * @version 0.2.0
 */

#include "MotorController.h"

// ============================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================

MotorController::MotorController(uint8_t pin_forward, uint8_t pin_backward,
                                 bool simulation_mode) :
    _pin_forward(pin_forward),
    _pin_backward(pin_backward),
    _current_speed(0.0f),
    _direction(MOTOR_COAST),
    _simulation_mode(simulation_mode),
    _initialized(false),
    _inverted(false),
    _deadband(5.0f),      // 5% default deadband
    _max_speed(100.0f)
{
}

MotorController::~MotorController() {
    stop(true);
}

// ============================================
// INITIALIZATION
// ============================================

bool MotorController::begin() {
    if (_initialized) {
        return true;
    }

    if (!_simulation_mode) {
        // Configure pins as outputs
        pinMode(_pin_forward, OUTPUT);
        pinMode(_pin_backward, OUTPUT);

        // Initialize to stopped state
        digitalWrite(_pin_forward, LOW);
        digitalWrite(_pin_backward, LOW);

        // On Portenta H7, PWM is available on most pins
        // analogWriteResolution() can be used if needed
    }

    _initialized = true;
    return true;
}

// ============================================
// SPEED CONTROL
// ============================================

void MotorController::set_speed(float speed) {
    if (!_initialized) {
        return;
    }

    // Constrain to valid range
    speed = _constrain_speed(speed);

    // Determine direction from sign
    if (speed > 0.0f) {
        _direction = MOTOR_FORWARD;
        _current_speed = speed;
    } else if (speed < 0.0f) {
        _direction = MOTOR_BACKWARD;
        _current_speed = -speed;  // Store as positive
    } else {
        _direction = MOTOR_COAST;
        _current_speed = 0.0f;
    }

    // Apply speed to hardware
    _apply_speed();
}

void MotorController::set_speed(float speed, MotorDirection direction) {
    if (!_initialized) {
        return;
    }

    // Constrain speed to 0-100
    speed = abs(_constrain_speed(speed));

    _current_speed = speed;
    _direction = direction;

    // Apply speed to hardware
    _apply_speed();
}

float MotorController::get_speed() const {
    // Return signed speed
    if (_direction == MOTOR_FORWARD) {
        return _current_speed;
    } else if (_direction == MOTOR_BACKWARD) {
        return -_current_speed;
    } else {
        return 0.0f;
    }
}

MotorDirection MotorController::get_direction() const {
    return _direction;
}

// ============================================
// STOP CONTROL
// ============================================

void MotorController::brake() {
    _current_speed = 0.0f;
    _direction = MOTOR_BRAKE;
    _apply_speed();
}

void MotorController::coast() {
    _current_speed = 0.0f;
    _direction = MOTOR_COAST;
    _apply_speed();
}

void MotorController::stop(bool use_brake) {
    if (use_brake) {
        brake();
    } else {
        coast();
    }
}

// ============================================
// STATUS
// ============================================

bool MotorController::is_moving() const {
    return (_current_speed > 0.0f) &&
           (_direction == MOTOR_FORWARD || _direction == MOTOR_BACKWARD);
}

// ============================================
// CONFIGURATION
// ============================================

void MotorController::set_deadband(float deadband) {
    if (deadband >= 0.0f && deadband <= 100.0f) {
        _deadband = deadband;
    }
}

float MotorController::get_deadband() const {
    return _deadband;
}

void MotorController::set_max_speed(float max_speed) {
    if (max_speed > 0.0f && max_speed <= 100.0f) {
        _max_speed = max_speed;
    }
}

float MotorController::get_max_speed() const {
    return _max_speed;
}

void MotorController::set_inverted(bool inverted) {
    _inverted = inverted;
}

bool MotorController::is_inverted() const {
    return _inverted;
}

// ============================================
// PRIVATE METHODS
// ============================================

void MotorController::_apply_speed() {
    if (_simulation_mode) {
        // In simulation mode, just update internal state
        return;
    }

    // Apply deadband
    float effective_speed = _current_speed;
    if (effective_speed > 0.0f && effective_speed < _deadband) {
        effective_speed = 0.0f;
    }

    // Convert to PWM value
    uint8_t pwm = _speed_to_pwm(effective_speed);

    // Determine actual direction (considering inversion)
    MotorDirection actual_direction = _direction;
    if (_inverted) {
        if (actual_direction == MOTOR_FORWARD) {
            actual_direction = MOTOR_BACKWARD;
        } else if (actual_direction == MOTOR_BACKWARD) {
            actual_direction = MOTOR_FORWARD;
        }
    }

    // Apply to pins based on direction
    switch (actual_direction) {
        case MOTOR_FORWARD:
            _set_pwm(_pin_forward, pwm);
            _set_pwm(_pin_backward, 0);
            break;

        case MOTOR_BACKWARD:
            _set_pwm(_pin_forward, 0);
            _set_pwm(_pin_backward, pwm);
            break;

        case MOTOR_BRAKE:
            // Both pins HIGH for active braking
            _set_pwm(_pin_forward, 255);
            _set_pwm(_pin_backward, 255);
            break;

        case MOTOR_COAST:
        default:
            // Both pins LOW to coast
            _set_pwm(_pin_forward, 0);
            _set_pwm(_pin_backward, 0);
            break;
    }
}

void MotorController::_set_pwm(uint8_t pin, uint8_t value) {
    if (value == 0) {
        digitalWrite(pin, LOW);
    } else if (value == 255) {
        digitalWrite(pin, HIGH);
    } else {
        analogWrite(pin, value);
    }
}

uint8_t MotorController::_speed_to_pwm(float speed_percent) {
    // Convert 0-100% to 0-255 PWM
    uint16_t pwm = (uint16_t)((speed_percent / 100.0f) * 255.0f);
    if (pwm > 255) pwm = 255;
    return (uint8_t)pwm;
}

float MotorController::_constrain_speed(float speed) {
    // Constrain to ±max_speed
    if (speed > _max_speed) {
        return _max_speed;
    } else if (speed < -_max_speed) {
        return -_max_speed;
    }
    return speed;
}
