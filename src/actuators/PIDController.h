/**
 * @file PIDController.h
 * @brief PID controller for motor speed regulation
 * @author Boris Mach
 * @version 0.2.0
 *
 * Proportional-Integral-Derivative controller for precise speed control.
 * Features:
 * - Configurable PID gains (Kp, Ki, Kd)
 * - Anti-windup protection
 * - Output limiting
 * - Derivative filter
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

/**
 * @brief PID Controller class
 *
 * Implements a discrete-time PID controller with anti-windup
 * and output saturation.
 */
class PIDController {
public:
    /**
     * @brief Constructor
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    PIDController(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f);

    /**
     * @brief Destructor
     */
    ~PIDController();

    /**
     * @brief Compute PID output
     * @param setpoint Desired value
     * @param measured Current measured value
     * @param dt Time delta in seconds
     * @return Control output
     */
    float compute(float setpoint, float measured, float dt);

    /**
     * @brief Reset PID controller state
     */
    void reset();

    /**
     * @brief Set PID gains
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void set_gains(float kp, float ki, float kd);

    /**
     * @brief Get proportional gain
     * @return Kp value
     */
    float get_kp() const;

    /**
     * @brief Get integral gain
     * @return Ki value
     */
    float get_ki() const;

    /**
     * @brief Get derivative gain
     * @return Kd value
     */
    float get_kd() const;

    /**
     * @brief Set output limits
     * @param min Minimum output value
     * @param max Maximum output value
     */
    void set_output_limits(float min, float max);

    /**
     * @brief Get minimum output limit
     * @return Min output
     */
    float get_output_min() const;

    /**
     * @brief Get maximum output limit
     * @return Max output
     */
    float get_output_max() const;

    /**
     * @brief Set integral windup limits
     * @param limit Maximum absolute integral term
     */
    void set_integral_limit(float limit);

    /**
     * @brief Get current error
     * @return Latest error value
     */
    float get_error() const;

    /**
     * @brief Get current integral term
     * @return Integral accumulator
     */
    float get_integral() const;

    /**
     * @brief Get current derivative term
     * @return Derivative value
     */
    float get_derivative() const;

    /**
     * @brief Enable/disable derivative filter
     * @param enable If true, use low-pass filter on derivative
     */
    void set_derivative_filter(bool enable);

    /**
     * @brief Set derivative filter coefficient (0-1)
     * @param alpha Filter coefficient (lower = more filtering)
     */
    void set_derivative_filter_alpha(float alpha);

private:
    // PID gains
    float _kp;
    float _ki;
    float _kd;

    // PID state
    float _error;
    float _prev_error;
    float _integral;
    float _derivative;
    float _prev_derivative;

    // Output limits
    float _output_min;
    float _output_max;

    // Integral limits (anti-windup)
    float _integral_limit;

    // Derivative filter
    bool _use_derivative_filter;
    float _derivative_filter_alpha;

    // Internal methods
    float _constrain_output(float output);
    float _constrain_integral(float integral);
};

#endif // PID_CONTROLLER_H
