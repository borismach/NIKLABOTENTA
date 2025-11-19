/**
 * @file MotorController.h
 * @brief DC Motor controller with PWM speed control
 * @author Boris Mach
 * @version 0.2.0
 *
 * This class provides control for a single DC motor with:
 * - PWM speed control (0-100%)
 * - Bidirectional control (forward/backward)
 * - Brake and coast modes
 * - Simulation mode for testing without hardware
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

/**
 * @brief Motor direction enumeration
 */
enum MotorDirection {
    MOTOR_FORWARD = 0,
    MOTOR_BACKWARD = 1,
    MOTOR_BRAKE = 2,
    MOTOR_COAST = 3
};

/**
 * @brief DC Motor controller class
 *
 * Controls a single DC motor using PWM for speed control.
 * Supports both H-bridge (2-pin) and direction+PWM (2-pin) configurations.
 */
class MotorController {
public:
    /**
     * @brief Constructor for H-bridge configuration
     * @param pin_forward Forward control pin (or IN1)
     * @param pin_backward Backward control pin (or IN2)
     * @param simulation_mode If true, no hardware control
     */
    MotorController(uint8_t pin_forward, uint8_t pin_backward,
                    bool simulation_mode = true);

    /**
     * @brief Destructor
     */
    ~MotorController();

    /**
     * @brief Initialize motor pins
     * @return true if successful
     */
    bool begin();

    /**
     * @brief Set motor speed and direction
     * @param speed Speed percentage (-100.0 to 100.0)
     *              Positive = forward, Negative = backward
     */
    void set_speed(float speed);

    /**
     * @brief Set motor speed with explicit direction
     * @param speed Speed percentage (0.0 to 100.0)
     * @param direction Motor direction (FORWARD/BACKWARD)
     */
    void set_speed(float speed, MotorDirection direction);

    /**
     * @brief Get current motor speed
     * @return Speed percentage (-100.0 to 100.0)
     */
    float get_speed() const;

    /**
     * @brief Get current motor direction
     * @return Current direction
     */
    MotorDirection get_direction() const;

    /**
     * @brief Apply active braking
     */
    void brake();

    /**
     * @brief Coast motor to stop (no braking)
     */
    void coast();

    /**
     * @brief Stop motor (uses brake by default)
     * @param use_brake If true, active brake; if false, coast
     */
    void stop(bool use_brake = true);

    /**
     * @brief Check if motor is moving
     * @return true if speed > 0
     */
    bool is_moving() const;

    /**
     * @brief Set speed deadband (minimum speed to overcome friction)
     * @param deadband Minimum speed percentage (0-100)
     */
    void set_deadband(float deadband);

    /**
     * @brief Get speed deadband
     * @return Deadband percentage
     */
    float get_deadband() const;

    /**
     * @brief Set maximum speed limit
     * @param max_speed Maximum speed percentage (0-100)
     */
    void set_max_speed(float max_speed);

    /**
     * @brief Get maximum speed limit
     * @return Maximum speed percentage
     */
    float get_max_speed() const;

    /**
     * @brief Invert motor direction (useful for wiring corrections)
     * @param inverted If true, reverse all commands
     */
    void set_inverted(bool inverted);

    /**
     * @brief Check if motor is inverted
     * @return true if inverted
     */
    bool is_inverted() const;

private:
    // Pin configuration
    uint8_t _pin_forward;
    uint8_t _pin_backward;

    // Motor state
    float _current_speed;        // -100.0 to 100.0
    MotorDirection _direction;

    // Configuration
    bool _simulation_mode;
    bool _initialized;
    bool _inverted;
    float _deadband;             // Minimum speed to overcome friction
    float _max_speed;            // Maximum allowed speed

    // PWM configuration
    static const uint16_t PWM_FREQUENCY = 1000;  // 1kHz
    static const uint16_t PWM_RESOLUTION = 8;     // 8-bit (0-255)

    // Internal methods
    void _apply_speed();
    void _set_pwm(uint8_t pin, uint8_t value);
    uint8_t _speed_to_pwm(float speed_percent);
    float _constrain_speed(float speed);
};

#endif // MOTOR_CONTROLLER_H
