/**
 * @file Encoder.cpp
 * @brief Implementation of Encoder class
 * @author Boris Mach
 * @version 0.2.0
 */

#include "Encoder.h"

// Static member initialization
Encoder* Encoder::_instances[2] = {nullptr, nullptr};
uint8_t Encoder::_instance_count = 0;

// ============================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================

Encoder::Encoder(uint8_t pin_a, uint8_t pin_b,
                 uint16_t ticks_per_rev,
                 float wheel_diameter_mm,
                 bool simulation_mode) :
    _pin_a(pin_a),
    _pin_b(pin_b),
    _ticks_per_rev(ticks_per_rev),
    _wheel_diameter_mm(wheel_diameter_mm),
    _tick_count(0),
    _prev_tick_count(0),
    _last_update_time(0),
    _current_velocity(0.0f),
    _simulated_velocity(0.0f),
    _simulation_mode(simulation_mode),
    _initialized(false),
    _instance_id(0)
{
    // Calculate wheel circumference in cm
    _wheel_circumference_cm = (PI * wheel_diameter_mm) / 10.0f;

    // Register instance for ISR
    if (_instance_count < 2) {
        _instance_id = _instance_count;
        _instances[_instance_count] = this;
        _instance_count++;
    }
}

Encoder::~Encoder() {
    // Detach interrupts if in hardware mode
    if (!_simulation_mode && _initialized) {
        detachInterrupt(digitalPinToInterrupt(_pin_a));
        detachInterrupt(digitalPinToInterrupt(_pin_b));
    }

    // Remove from instance array
    if (_instance_id < 2 && _instances[_instance_id] == this) {
        _instances[_instance_id] = nullptr;
    }
}

// ============================================
// INITIALIZATION
// ============================================

bool Encoder::begin() {
    if (_initialized) {
        return true;
    }

    if (!_simulation_mode) {
        // Configure pins as inputs with pullup
        pinMode(_pin_a, INPUT_PULLUP);
        pinMode(_pin_b, INPUT_PULLUP);

        // Attach interrupts (both edges for better resolution)
        // Note: On Portenta H7, most pins support interrupts
        attachInterrupt(digitalPinToInterrupt(_pin_a),
                       _isr_handler_a, CHANGE);
        attachInterrupt(digitalPinToInterrupt(_pin_b),
                       _isr_handler_b, CHANGE);
    }

    _last_update_time = millis();
    _initialized = true;

    return true;
}

// ============================================
// UPDATE
// ============================================

void Encoder::update() {
    if (!_initialized) {
        return;
    }

    unsigned long current_time = millis();
    unsigned long dt = current_time - _last_update_time;

    // Update velocity at regular intervals
    if (dt >= UPDATE_INTERVAL_MS) {
        _calculate_velocity();
        _last_update_time = current_time;
    }

    // In simulation mode, update tick count based on simulated velocity
    if (_simulation_mode && _simulated_velocity != 0.0f) {
        float dt_sec = dt / 1000.0f;
        float distance_cm = _simulated_velocity * dt_sec;
        int32_t ticks = (int32_t)((distance_cm / _wheel_circumference_cm) * _ticks_per_rev);
        _tick_count += ticks;
    }
}

// ============================================
// TICK COUNT
// ============================================

int32_t Encoder::get_ticks() const {
    return _tick_count;
}

void Encoder::reset_ticks() {
    _tick_count = 0;
    _prev_tick_count = 0;
}

// ============================================
// VELOCITY
// ============================================

float Encoder::get_velocity() const {
    if (_simulation_mode) {
        return _simulated_velocity;
    }
    return _current_velocity;
}

float Encoder::get_rpm() const {
    // Calculate RPM from velocity
    // velocity (cm/s) → rev/s → rev/min
    float rev_per_sec = (_current_velocity / _wheel_circumference_cm);
    return rev_per_sec * 60.0f;
}

// ============================================
// DISTANCE
// ============================================

float Encoder::get_distance() const {
    return abs(_ticks_to_distance(_tick_count));
}

float Encoder::get_signed_distance() const {
    return _ticks_to_distance(_tick_count);
}

void Encoder::reset_distance() {
    reset_ticks();
}

// ============================================
// SIMULATION
// ============================================

void Encoder::set_simulated_velocity(float velocity_cm_s) {
    if (_simulation_mode) {
        _simulated_velocity = velocity_cm_s;
    }
}

void Encoder::simulate_ticks(int32_t ticks) {
    if (_simulation_mode) {
        _tick_count += ticks;
    }
}

// ============================================
// GETTERS
// ============================================

uint16_t Encoder::get_ticks_per_rev() const {
    return _ticks_per_rev;
}

float Encoder::get_wheel_diameter() const {
    return _wheel_diameter_mm;
}

// ============================================
// PRIVATE METHODS
// ============================================

void Encoder::_calculate_velocity() {
    // Calculate velocity based on tick delta
    int32_t tick_delta = _tick_count - _prev_tick_count;
    _prev_tick_count = _tick_count;

    // Convert ticks to distance
    float distance_cm = _ticks_to_distance(tick_delta);

    // Calculate velocity (distance / time)
    float dt_sec = UPDATE_INTERVAL_MS / 1000.0f;
    _current_velocity = distance_cm / dt_sec;
}

float Encoder::_ticks_to_distance(int32_t ticks) const {
    // Convert ticks to distance in cm
    float revolutions = (float)ticks / (float)_ticks_per_rev;
    return revolutions * _wheel_circumference_cm;
}

// ============================================
// INTERRUPT HANDLERS
// ============================================

void Encoder::_isr_handler_a() {
    // Handle channel A interrupt for first encoder
    if (_instances[0] != nullptr) {
        // Simple tick counting (can be enhanced with quadrature decoding)
        bool a_state = digitalRead(_instances[0]->_pin_a);
        bool b_state = digitalRead(_instances[0]->_pin_b);

        // Determine direction based on A/B states
        if (a_state == b_state) {
            _instances[0]->_tick_count++;
        } else {
            _instances[0]->_tick_count--;
        }
    }
}

void Encoder::_isr_handler_b() {
    // Handle channel B interrupt for second encoder
    if (_instances[1] != nullptr) {
        // Simple tick counting
        bool a_state = digitalRead(_instances[1]->_pin_a);
        bool b_state = digitalRead(_instances[1]->_pin_b);

        // Determine direction
        if (a_state == b_state) {
            _instances[1]->_tick_count++;
        } else {
            _instances[1]->_tick_count--;
        }
    }
}
