/**
 * @file NiclaVision_Interface.cpp
 * @brief Implementation of Nicla Vision camera interface
 * @author Boris Mach
 * @version 0.4.0
 */

#include "NiclaVision_Interface.h"
#include <math.h>

// ============================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================

NiclaVision_Interface::NiclaVision_Interface(bool simulation_mode) :
    _simulation_mode(simulation_mode),
    _initialized(false),
    _frame_width(320),
    _frame_height(240),
    _line_threshold(128),
    _color_tolerance(20),
    _proximity(100.0f),  // Far away
    _brightness(50),
    _contrast(50)
{
    // Initialize line detection state
    _line.detected = false;
    _line.position = 0.0f;
    _line.angle = 0.0f;
    _line.thickness = 0;
    _line.confidence = 0;

    // Initialize color detection state
    _color.detected = false;
    _color.r = 128;
    _color.g = 128;
    _color.b = 128;
    _color.hue = 0.0f;
    _color.saturation = 0.0f;
    _color.value = 0.5f;
    _color.name = "gray";

    // Initialize object detection state
    _object.detected = false;
    _object.x = _frame_width / 2;
    _object.y = _frame_height / 2;
    _object.width = 0;
    _object.height = 0;
    _object.distance = 0.0f;
}

NiclaVision_Interface::~NiclaVision_Interface() {
    // Cleanup if needed
}

// ============================================
// INITIALIZATION
// ============================================

bool NiclaVision_Interface::begin() {
    if (_initialized) {
        return true;
    }

    if (_simulation_mode) {
        // Simulation mode - always succeeds
        _initialized = true;
        return true;
    }

#ifndef SIMULATION_ONLY
    // Hardware mode - initialize camera
    // TODO: Initialize Nicla Vision camera
    // camera.begin(CAMERA_R320x240, CAMERA_RGB565, 30);

    // For now, return false if not in simulation
    return false;
#else
    // If compiled without camera library, can't use hardware
    return false;
#endif
}

// ============================================
// UPDATE
// ============================================

void NiclaVision_Interface::update() {
    if (!_initialized) {
        return;
    }

    if (_simulation_mode) {
        // In simulation, update simulated sensors
        // Line detection slowly changes position (simulating movement)
        static unsigned long last_update = 0;
        if (millis() - last_update > 100) {
            // Simulate line wobbling
            _line.position += (random(-10, 11) / 100.0f);
            if (_line.position > 1.0f) _line.position = 1.0f;
            if (_line.position < -1.0f) _line.position = -1.0f;

            last_update = millis();
        }

        return;
    }

#ifndef SIMULATION_ONLY
    // Hardware mode - capture and process frame
    // TODO: Capture frame and run detection algorithms
#endif
}

// ============================================
// LINE DETECTION
// ============================================

LineDetection NiclaVision_Interface::detect_line() {
    if (!_initialized) {
        return _line;
    }

    if (_simulation_mode) {
        // Return simulated line
        return _line;
    }

    // Hardware mode: process camera frame for line
    _process_line_detection();

    return _line;
}

bool NiclaVision_Interface::is_line_detected() {
    return _line.detected;
}

float NiclaVision_Interface::get_line_position() {
    return _line.position;
}

float NiclaVision_Interface::get_line_angle() {
    return _line.angle;
}

void NiclaVision_Interface::set_line_threshold(uint8_t threshold) {
    _line_threshold = threshold;
}

// ============================================
// COLOR DETECTION
// ============================================

ColorDetection NiclaVision_Interface::detect_color() {
    if (!_initialized) {
        return _color;
    }

    if (_simulation_mode) {
        // Return simulated color
        return _color;
    }

    // Hardware mode: process camera frame for color
    _process_color_detection();

    return _color;
}

bool NiclaVision_Interface::detect_color(const String& color_name) {
    ColorDetection color = detect_color();

    // Simple color name matching
    return color.detected && color.name.equalsIgnoreCase(color_name);
}

bool NiclaVision_Interface::get_detected_color(uint8_t& r, uint8_t& g, uint8_t& b) {
    if (_color.detected) {
        r = _color.r;
        g = _color.g;
        b = _color.b;
        return true;
    }

    return false;
}

void NiclaVision_Interface::set_color_tolerance(uint8_t tolerance) {
    _color_tolerance = constrain(tolerance, 0, 100);
}

// ============================================
// OBJECT DETECTION
// ============================================

ObjectDetection NiclaVision_Interface::detect_object() {
    if (!_initialized) {
        return _object;
    }

    if (_simulation_mode) {
        // Return simulated object
        return _object;
    }

    // Hardware mode: process camera frame for objects
    _process_object_detection();

    return _object;
}

bool NiclaVision_Interface::is_object_detected() {
    return _object.detected;
}

float NiclaVision_Interface::get_object_distance() {
    if (_object.detected) {
        return _object.distance;
    }

    return get_proximity();  // Fallback to proximity sensor
}

// ============================================
// PROXIMITY SENSOR
// ============================================

float NiclaVision_Interface::get_proximity() {
    if (!_initialized) {
        return 100.0f;  // Far away
    }

    if (_simulation_mode) {
        return _proximity;
    }

    // Hardware mode: read proximity sensor
    _update_proximity_sensor();

    return _proximity;
}

bool NiclaVision_Interface::is_object_close(float threshold) {
    return get_proximity() < threshold;
}

// ============================================
// CAMERA CONTROL
// ============================================

bool NiclaVision_Interface::capture_frame() {
    if (!_initialized) {
        return false;
    }

    if (_simulation_mode) {
        // Simulate successful capture
        return true;
    }

#ifndef SIMULATION_ONLY
    // Hardware mode: capture frame from camera
    // TODO: Implement frame capture
    return false;
#else
    return false;
#endif
}

uint16_t NiclaVision_Interface::get_frame_width() const {
    return _frame_width;
}

uint16_t NiclaVision_Interface::get_frame_height() const {
    return _frame_height;
}

void NiclaVision_Interface::set_brightness(uint8_t brightness) {
    _brightness = constrain(brightness, 0, 100);

#ifndef SIMULATION_ONLY
    if (!_simulation_mode) {
        // TODO: Set camera brightness
    }
#endif
}

void NiclaVision_Interface::set_contrast(uint8_t contrast) {
    _contrast = constrain(contrast, 0, 100);

#ifndef SIMULATION_ONLY
    if (!_simulation_mode) {
        // TODO: Set camera contrast
    }
#endif
}

// ============================================
// CONFIGURATION
// ============================================

bool NiclaVision_Interface::is_initialized() const {
    return _initialized;
}

bool NiclaVision_Interface::is_simulation_mode() const {
    return _simulation_mode;
}

void NiclaVision_Interface::set_simulated_line(bool detected, float position) {
    if (_simulation_mode) {
        _line.detected = detected;
        _line.position = constrain(position, -1.0f, 1.0f);
        _line.angle = 0.0f;
        _line.thickness = detected ? 10 : 0;
        _line.confidence = detected ? 85 : 0;
    }
}

void NiclaVision_Interface::set_simulated_color(uint8_t r, uint8_t g, uint8_t b) {
    if (_simulation_mode) {
        _color.r = r;
        _color.g = g;
        _color.b = b;
        _color.detected = true;

        // Convert to HSV
        _rgb_to_hsv(r, g, b, _color.hue, _color.saturation, _color.value);

        // Get color name
        _color.name = _get_color_name(_color.hue, _color.saturation, _color.value);
    }
}

// ============================================
// PRIVATE METHODS
// ============================================

void NiclaVision_Interface::_process_line_detection() {
    // Hardware implementation would:
    // 1. Threshold image to binary
    // 2. Find largest contour/blob
    // 3. Calculate centroid and angle
    // 4. Convert to position (-1 to 1)

    // Placeholder for hardware implementation
    _line.detected = false;
    _line.position = 0.0f;
    _line.angle = 0.0f;
    _line.thickness = 0;
    _line.confidence = 0;
}

void NiclaVision_Interface::_process_color_detection() {
    // Hardware implementation would:
    // 1. Calculate average color in center region
    // 2. Convert RGB to HSV
    // 3. Determine dominant color name

    // Placeholder for hardware implementation
    _color.detected = false;
    _color.r = 128;
    _color.g = 128;
    _color.b = 128;
}

void NiclaVision_Interface::_process_object_detection() {
    // Hardware implementation would:
    // 1. Run blob detection or simple thresholding
    // 2. Find largest blob
    // 3. Calculate bounding box
    // 4. Estimate distance if possible

    // Placeholder for hardware implementation
    _object.detected = false;
}

void NiclaVision_Interface::_update_proximity_sensor() {
    // Hardware implementation would:
    // Read proximity sensor value
    // Convert to distance in cm

    // Placeholder for hardware implementation
    _proximity = 100.0f;
}

void NiclaVision_Interface::_rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b,
                                        float& h, float& s, float& v) {
    // Convert RGB (0-255) to HSV (H: 0-360, S: 0-1, V: 0-1)
    float rf = r / 255.0f;
    float gf = g / 255.0f;
    float bf = b / 255.0f;

    float max_val = max(max(rf, gf), bf);
    float min_val = min(min(rf, gf), bf);
    float delta = max_val - min_val;

    // Value
    v = max_val;

    // Saturation
    if (max_val > 0.0f) {
        s = delta / max_val;
    } else {
        s = 0.0f;
    }

    // Hue
    if (delta == 0.0f) {
        h = 0.0f;  // Undefined, but set to 0
    } else if (max_val == rf) {
        h = 60.0f * fmod(((gf - bf) / delta), 6.0f);
    } else if (max_val == gf) {
        h = 60.0f * (((bf - rf) / delta) + 2.0f);
    } else {
        h = 60.0f * (((rf - gf) / delta) + 4.0f);
    }

    // Normalize hue to 0-360
    if (h < 0.0f) {
        h += 360.0f;
    }
}

String NiclaVision_Interface::_get_color_name(float hue, float saturation, float value) {
    // Simple color naming based on HSV
    // Low saturation or value = gray/black/white
    if (saturation < 0.2f) {
        if (value < 0.3f) return "black";
        if (value > 0.7f) return "white";
        return "gray";
    }

    // Chromatic colors based on hue
    if (hue < 30 || hue >= 330) return "red";
    if (hue < 60) return "orange";
    if (hue < 90) return "yellow";
    if (hue < 150) return "green";
    if (hue < 210) return "cyan";
    if (hue < 270) return "blue";
    if (hue < 330) return "magenta";

    return "unknown";
}
