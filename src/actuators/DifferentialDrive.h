/**
 * @file DifferentialDrive.h
 * @brief Differential drive kinematics and control
 * @author Boris Mach
 * @version 0.2.0
 *
 * This class implements differential drive robot kinematics with:
 * - Forward/inverse kinematics
 * - Odometry tracking
 * - Coordinated motor control
 * - PID-based speed regulation
 * - Both simulation and hardware modes
 */

#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include <Arduino.h>
#include "MotorController.h"
#include "Encoder.h"
#include "PIDController.h"

/**
 * @brief Robot pose structure (position + orientation)
 */
struct Pose {
    float x;        // X position in cm
    float y;        // Y position in cm
    float theta;    // Heading in radians
};

/**
 * @brief Differential drive controller class
 *
 * Manages a 2-wheel differential drive robot with:
 * - Two DC motors with encoders
 * - PID speed control
 * - Odometry calculation
 * - High-level movement commands
 */
class DifferentialDrive {
public:
    /**
     * @brief Constructor
     * @param wheelbase_cm Distance between wheels in cm
     * @param wheel_diameter_mm Wheel diameter in mm
     * @param simulation_mode If true, run without hardware
     */
    DifferentialDrive(float wheelbase_cm,
                     float wheel_diameter_mm,
                     bool simulation_mode = true);

    /**
     * @brief Destructor
     */
    ~DifferentialDrive();

    /**
     * @brief Initialize differential drive system
     * @param left_motor_fwd Left motor forward pin
     * @param left_motor_bwd Left motor backward pin
     * @param right_motor_fwd Right motor forward pin
     * @param right_motor_bwd Right motor backward pin
     * @param left_enc_a Left encoder channel A
     * @param left_enc_b Left encoder channel B
     * @param right_enc_a Right encoder channel A
     * @param right_enc_b Right encoder channel B
     * @param encoder_ticks_per_rev Encoder resolution
     * @return true if successful
     */
    bool begin(uint8_t left_motor_fwd, uint8_t left_motor_bwd,
               uint8_t right_motor_fwd, uint8_t right_motor_bwd,
               uint8_t left_enc_a, uint8_t left_enc_b,
               uint8_t right_enc_a, uint8_t right_enc_b,
               uint16_t encoder_ticks_per_rev);

    /**
     * @brief Update odometry and PID controllers
     * Must be called regularly (e.g., in loop())
     */
    void update();

    /**
     * @brief Drive with linear and angular velocity
     * @param linear_velocity Linear speed in cm/s
     * @param angular_velocity Angular speed in rad/s (positive = turn right)
     */
    void drive(float linear_velocity, float angular_velocity);

    /**
     * @brief Set individual wheel velocities
     * @param left_velocity Left wheel speed in cm/s
     * @param right_velocity Right wheel speed in cm/s
     */
    void set_wheel_velocities(float left_velocity, float right_velocity);

    /**
     * @brief Move forward/backward by distance
     * @param distance_cm Distance in cm (positive = forward)
     * @param speed_cm_s Travel speed in cm/s
     * @param blocking If true, wait until complete
     * @return true if movement complete (non-blocking mode)
     */
    bool move_distance(float distance_cm, float speed_cm_s, bool blocking = true);

    /**
     * @brief Rotate by angle
     * @param angle_rad Angle in radians (positive = clockwise)
     * @param speed_rad_s Angular speed in rad/s
     * @param blocking If true, wait until complete
     * @return true if rotation complete (non-blocking mode)
     */
    bool rotate_angle(float angle_rad, float speed_rad_s, bool blocking = true);

    /**
     * @brief Stop both motors
     * @param use_brake If true, active brake; if false, coast
     */
    void stop(bool use_brake = true);

    /**
     * @brief Get current robot pose
     * @return Pose structure (x, y, theta)
     */
    Pose get_pose() const;

    /**
     * @brief Reset pose to origin
     * @param x Initial X position
     * @param y Initial Y position
     * @param theta Initial heading
     */
    void reset_pose(float x = 0.0f, float y = 0.0f, float theta = 0.0f);

    /**
     * @brief Get left wheel velocity
     * @return Velocity in cm/s
     */
    float get_left_velocity() const;

    /**
     * @brief Get right wheel velocity
     * @return Velocity in cm/s
     */
    float get_right_velocity() const;

    /**
     * @brief Get robot linear velocity
     * @return Linear velocity in cm/s
     */
    float get_linear_velocity() const;

    /**
     * @brief Get robot angular velocity
     * @return Angular velocity in rad/s
     */
    float get_angular_velocity() const;

    /**
     * @brief Set PID gains for both wheels
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void set_pid_gains(float kp, float ki, float kd);

    /**
     * @brief Enable/disable PID control
     * @param enable If false, use open-loop control
     */
    void enable_pid(bool enable);

    /**
     * @brief Check if PID is enabled
     * @return true if PID enabled
     */
    bool is_pid_enabled() const;

    /**
     * @brief Set motor direction inversion
     * @param left_inverted Invert left motor
     * @param right_inverted Invert right motor
     */
    void set_motor_inversion(bool left_inverted, bool right_inverted);

private:
    // Robot parameters
    float _wheelbase_cm;
    float _wheel_diameter_mm;
    float _wheel_radius_cm;

    // Components (pointers for optional hardware)
    MotorController* _left_motor;
    MotorController* _right_motor;
    Encoder* _left_encoder;
    Encoder* _right_encoder;
    PIDController* _left_pid;
    PIDController* _right_pid;

    // State
    Pose _pose;
    float _left_velocity;        // cm/s
    float _right_velocity;       // cm/s
    unsigned long _last_update_time;

    // Configuration
    bool _simulation_mode;
    bool _initialized;
    bool _pid_enabled;

    // Movement command state (for non-blocking moves)
    enum MoveState {
        MOVE_IDLE,
        MOVE_DISTANCE,
        MOVE_ROTATE
    };
    MoveState _move_state;
    float _move_target;
    float _move_start_value;

    // Update interval
    static const unsigned long UPDATE_INTERVAL_MS = 20;  // 50 Hz

    // Internal methods
    void _update_odometry(float dt);
    void _update_pid_control(float dt);
    void _inverse_kinematics(float linear_vel, float angular_vel,
                            float& left_vel, float& right_vel);
    void _forward_kinematics(float left_vel, float right_vel,
                            float& linear_vel, float& angular_vel);
};

#endif // DIFFERENTIAL_DRIVE_H
