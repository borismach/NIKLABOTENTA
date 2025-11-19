/**
 * @file PIDController.cpp
 * @brief Implementation of PID controller
 * @author Boris Mach
 * @version 0.2.0
 */

#include "PIDController.h"

// ============================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================

PIDController::PIDController(float kp, float ki, float kd) :
    _kp(kp),
    _ki(ki),
    _kd(kd),
    _error(0.0f),
    _prev_error(0.0f),
    _integral(0.0f),
    _derivative(0.0f),
    _prev_derivative(0.0f),
    _output_min(-100.0f),
    _output_max(100.0f),
    _integral_limit(50.0f),
    _use_derivative_filter(true),
    _derivative_filter_alpha(0.1f)
{
}

PIDController::~PIDController() {
}

// ============================================
// COMPUTATION
// ============================================

float PIDController::compute(float setpoint, float measured, float dt) {
    // Avoid division by zero
    if (dt <= 0.0f) {
        return 0.0f;
    }

    // Calculate error
    _error = setpoint - measured;

    // Proportional term
    float p_term = _kp * _error;

    // Integral term (with anti-windup)
    _integral += _error * dt;
    _integral = _constrain_integral(_integral);
    float i_term = _ki * _integral;

    // Derivative term (derivative of error)
    float derivative_raw = (_error - _prev_error) / dt;

    // Apply low-pass filter to derivative if enabled
    if (_use_derivative_filter) {
        _derivative = _derivative_filter_alpha * derivative_raw +
                     (1.0f - _derivative_filter_alpha) * _prev_derivative;
        _prev_derivative = _derivative;
    } else {
        _derivative = derivative_raw;
    }

    float d_term = _kd * _derivative;

    // Calculate total output
    float output = p_term + i_term + d_term;

    // Constrain output
    output = _constrain_output(output);

    // Save error for next iteration
    _prev_error = _error;

    return output;
}

// ============================================
// RESET
// ============================================

void PIDController::reset() {
    _error = 0.0f;
    _prev_error = 0.0f;
    _integral = 0.0f;
    _derivative = 0.0f;
    _prev_derivative = 0.0f;
}

// ============================================
// CONFIGURATION
// ============================================

void PIDController::set_gains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

float PIDController::get_kp() const {
    return _kp;
}

float PIDController::get_ki() const {
    return _ki;
}

float PIDController::get_kd() const {
    return _kd;
}

void PIDController::set_output_limits(float min, float max) {
    if (min < max) {
        _output_min = min;
        _output_max = max;
    }
}

float PIDController::get_output_min() const {
    return _output_min;
}

float PIDController::get_output_max() const {
    return _output_max;
}

void PIDController::set_integral_limit(float limit) {
    if (limit > 0.0f) {
        _integral_limit = limit;
    }
}

// ============================================
// GETTERS
// ============================================

float PIDController::get_error() const {
    return _error;
}

float PIDController::get_integral() const {
    return _integral;
}

float PIDController::get_derivative() const {
    return _derivative;
}

// ============================================
// DERIVATIVE FILTER
// ============================================

void PIDController::set_derivative_filter(bool enable) {
    _use_derivative_filter = enable;
}

void PIDController::set_derivative_filter_alpha(float alpha) {
    // Constrain alpha to 0-1 range
    if (alpha >= 0.0f && alpha <= 1.0f) {
        _derivative_filter_alpha = alpha;
    }
}

// ============================================
// PRIVATE METHODS
// ============================================

float PIDController::_constrain_output(float output) {
    if (output > _output_max) {
        return _output_max;
    } else if (output < _output_min) {
        return _output_min;
    }
    return output;
}

float PIDController::_constrain_integral(float integral) {
    if (integral > _integral_limit) {
        return _integral_limit;
    } else if (integral < -_integral_limit) {
        return -_integral_limit;
    }
    return integral;
}
