/**
 * @file NiclaVision_Interface.h
 * @brief Interface for Arduino Nicla Vision camera and sensors
 * @author Boris Mach
 * @version 0.4.0
 *
 * This class provides a simplified interface to Nicla Vision with:
 * - Line detection for line following
 * - Color detection and recognition
 * - Basic object detection
 * - Proximity sensing
 * - Simulation mode for testing without hardware
 *
 * Note: Nicla Vision uses OpenMV-compatible libraries
 */

#ifndef NICLA_VISION_INTERFACE_H
#define NICLA_VISION_INTERFACE_H

#include <Arduino.h>

// Forward declarations for Nicla Vision libraries (only if hardware present)
#ifndef SIMULATION_ONLY
// Camera library would be included here
// #include <camera.h>
#endif

/**
 * @brief Line detection result structure
 */
struct LineDetection {
    bool detected;        // true if line found
    float position;       // Line position: -1.0 (left) to 1.0 (right), 0 = center
    float angle;          // Line angle in degrees (-90 to 90)
    uint8_t thickness;    // Estimated line thickness in pixels
    uint8_t confidence;   // Detection confidence (0-100)
};

/**
 * @brief Color detection result structure
 */
struct ColorDetection {
    bool detected;        // true if color found
    uint8_t r, g, b;     // RGB color values (0-255)
    float hue;           // Hue angle (0-360)
    float saturation;    // Saturation (0-1)
    float value;         // Value/brightness (0-1)
    String name;         // Color name (red, green, blue, etc.)
};

/**
 * @brief Object detection result structure
 */
struct ObjectDetection {
    bool detected;        // true if object found
    uint16_t x, y;       // Object center coordinates
    uint16_t width;      // Object width in pixels
    uint16_t height;     // Object height in pixels
    float distance;      // Estimated distance in cm (if available)
};

/**
 * @brief Nicla Vision camera interface class
 *
 * Provides simplified access to Nicla Vision camera and sensors.
 * Supports both hardware mode and simulation mode for testing.
 */
class NiclaVision_Interface {
public:
    /**
     * @brief Constructor
     * @param simulation_mode If true, run without hardware
     */
    NiclaVision_Interface(bool simulation_mode = true);

    /**
     * @brief Destructor
     */
    ~NiclaVision_Interface();

    /**
     * @brief Initialize the camera and sensors
     * @return true if successful
     */
    bool begin();

    /**
     * @brief Update sensor readings (call in loop)
     */
    void update();

    // ============================================
    // LINE DETECTION
    // ============================================

    /**
     * @brief Detect line in current frame
     * @return LineDetection structure with results
     */
    LineDetection detect_line();

    /**
     * @brief Check if line is detected
     * @return true if line found
     */
    bool is_line_detected();

    /**
     * @brief Get line position
     * @return Position from -1.0 (left) to 1.0 (right), 0 = center
     */
    float get_line_position();

    /**
     * @brief Get line angle
     * @return Angle in degrees (-90 to 90)
     */
    float get_line_angle();

    /**
     * @brief Set line detection sensitivity
     * @param threshold Detection threshold (0-255, lower = more sensitive)
     */
    void set_line_threshold(uint8_t threshold);

    // ============================================
    // COLOR DETECTION
    // ============================================

    /**
     * @brief Detect dominant color in frame
     * @return ColorDetection structure with results
     */
    ColorDetection detect_color();

    /**
     * @brief Detect specific color by name
     * @param color_name Color to detect (red, green, blue, yellow, etc.)
     * @return true if specified color found
     */
    bool detect_color(const String& color_name);

    /**
     * @brief Get detected RGB color
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     * @return true if color detected
     */
    bool get_detected_color(uint8_t& r, uint8_t& g, uint8_t& b);

    /**
     * @brief Set color detection tolerance
     * @param tolerance Tolerance value (0-100, higher = more tolerant)
     */
    void set_color_tolerance(uint8_t tolerance);

    // ============================================
    // OBJECT DETECTION
    // ============================================

    /**
     * @brief Detect object in frame
     * @return ObjectDetection structure with results
     */
    ObjectDetection detect_object();

    /**
     * @brief Check if object is detected
     * @return true if object found
     */
    bool is_object_detected();

    /**
     * @brief Get distance to detected object
     * @return Distance in centimeters (using proximity sensor)
     */
    float get_object_distance();

    // ============================================
    // PROXIMITY SENSOR
    // ============================================

    /**
     * @brief Get proximity sensor reading
     * @return Distance in centimeters (0-30 cm typical range)
     */
    float get_proximity();

    /**
     * @brief Check if object is close (< threshold)
     * @param threshold Distance threshold in cm
     * @return true if object within threshold
     */
    bool is_object_close(float threshold = 10.0f);

    // ============================================
    // CAMERA CONTROL
    // ============================================

    /**
     * @brief Capture a frame
     * @return true if capture successful
     */
    bool capture_frame();

    /**
     * @brief Get frame width
     * @return Frame width in pixels
     */
    uint16_t get_frame_width() const;

    /**
     * @brief Get frame height
     * @return Frame height in pixels
     */
    uint16_t get_frame_height() const;

    /**
     * @brief Set camera brightness
     * @param brightness Brightness level (0-100)
     */
    void set_brightness(uint8_t brightness);

    /**
     * @brief Set camera contrast
     * @param contrast Contrast level (0-100)
     */
    void set_contrast(uint8_t contrast);

    // ============================================
    // CONFIGURATION
    // ============================================

    /**
     * @brief Check if camera is initialized
     * @return true if initialized
     */
    bool is_initialized() const;

    /**
     * @brief Check if in simulation mode
     * @return true if simulation mode
     */
    bool is_simulation_mode() const;

    /**
     * @brief Set simulated line detection (for testing)
     * @param detected true if line present
     * @param position Line position (-1 to 1)
     */
    void set_simulated_line(bool detected, float position);

    /**
     * @brief Set simulated color detection (for testing)
     * @param r Red component
     * @param g Green component
     * @param b Blue component
     */
    void set_simulated_color(uint8_t r, uint8_t g, uint8_t b);

private:
    // Configuration
    bool _simulation_mode;
    bool _initialized;

    // Frame properties
    uint16_t _frame_width;
    uint16_t _frame_height;

    // Line detection state
    LineDetection _line;
    uint8_t _line_threshold;

    // Color detection state
    ColorDetection _color;
    uint8_t _color_tolerance;

    // Object detection state
    ObjectDetection _object;

    // Proximity sensor
    float _proximity;

    // Camera settings
    uint8_t _brightness;
    uint8_t _contrast;

    // Internal methods
    void _process_line_detection();
    void _process_color_detection();
    void _process_object_detection();
    void _update_proximity_sensor();
    void _rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b,
                     float& h, float& s, float& v);
    String _get_color_name(float hue, float saturation, float value);
};

#endif // NICLA_VISION_INTERFACE_H
