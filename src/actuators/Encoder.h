/**
 * @file Encoder.h
 * @brief Encoder reading and velocity calculation
 * @author Boris Mach
 * @version 0.2.0
 *
 * This class handles rotary encoder reading with:
 * - Interrupt-based tick counting
 * - Velocity calculation
 * - Distance measurement
 * - Direction detection
 * - Simulation mode for testing
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

/**
 * @brief Encoder reading class
 *
 * Reads quadrature encoder signals and calculates velocity/distance.
 * Supports both real hardware (interrupt-based) and simulation mode.
 */
class Encoder {
public:
    /**
     * @brief Constructor
     * @param pin_a Encoder channel A pin (interrupt capable)
     * @param pin_b Encoder channel B pin (interrupt capable)
     * @param ticks_per_rev Encoder ticks per revolution
     * @param wheel_diameter_mm Wheel diameter in millimeters
     * @param simulation_mode If true, no hardware reading
     */
    Encoder(uint8_t pin_a, uint8_t pin_b,
            uint16_t ticks_per_rev,
            float wheel_diameter_mm,
            bool simulation_mode = true);

    /**
     * @brief Destructor
     */
    ~Encoder();

    /**
     * @brief Initialize encoder
     * @return true if successful
     */
    bool begin();

    /**
     * @brief Update encoder readings (call in loop)
     * Should be called at regular intervals for velocity calculation
     */
    void update();

    /**
     * @brief Get current tick count
     * @return Total ticks (can be negative if reversed)
     */
    int32_t get_ticks() const;

    /**
     * @brief Reset tick count to zero
     */
    void reset_ticks();

    /**
     * @brief Get current velocity in cm/s
     * @return Velocity (positive = forward, negative = backward)
     */
    float get_velocity() const;

    /**
     * @brief Get total distance traveled in cm
     * @return Distance (always positive)
     */
    float get_distance() const;

    /**
     * @brief Get distance with sign (direction)
     * @return Signed distance in cm
     */
    float get_signed_distance() const;

    /**
     * @brief Reset distance to zero
     */
    void reset_distance();

    /**
     * @brief Get current RPM (revolutions per minute)
     * @return RPM value
     */
    float get_rpm() const;

    /**
     * @brief Set simulated velocity (for testing)
     * @param velocity_cm_s Velocity in cm/s
     * Only works in simulation mode
     */
    void set_simulated_velocity(float velocity_cm_s);

    /**
     * @brief Simulate tick increment (for testing)
     * @param ticks Number of ticks to add (can be negative)
     * Only works in simulation mode
     */
    void simulate_ticks(int32_t ticks);

    /**
     * @brief Get ticks per revolution
     * @return Ticks per rev
     */
    uint16_t get_ticks_per_rev() const;

    /**
     * @brief Get wheel diameter
     * @return Diameter in mm
     */
    float get_wheel_diameter() const;

private:
    // Pin configuration
    uint8_t _pin_a;
    uint8_t _pin_b;

    // Encoder parameters
    uint16_t _ticks_per_rev;
    float _wheel_diameter_mm;
    float _wheel_circumference_cm;

    // Encoder state
    volatile int32_t _tick_count;
    int32_t _prev_tick_count;

    // Velocity calculation
    unsigned long _last_update_time;
    float _current_velocity;     // cm/s
    float _simulated_velocity;   // For simulation mode

    // Configuration
    bool _simulation_mode;
    bool _initialized;

    // Update interval for velocity calculation
    static const unsigned long UPDATE_INTERVAL_MS = 50;  // 20 Hz

    // Internal methods
    void _calculate_velocity();
    float _ticks_to_distance(int32_t ticks) const;

    // Interrupt handlers (static for ISR)
    static void _isr_handler_a();
    static void _isr_handler_b();

    // Static instance pointers for ISR (max 2 encoders for typical robot)
    static Encoder* _instances[2];
    static uint8_t _instance_count;
    uint8_t _instance_id;
};

#endif // ENCODER_H
