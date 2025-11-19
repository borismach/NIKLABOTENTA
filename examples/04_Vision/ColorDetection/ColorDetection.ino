/**
 * @file ColorDetection.ino
 * @brief Color detection and recognition using Nicla Vision camera
 *
 * This example shows:
 * - Real-time color detection
 * - RGB values and HSV conversion
 * - Color name identification
 * - Visual representation of detected colors
 * - Reactive behavior based on colors
 *
 * Features:
 * - Continuous color sampling
 * - Named color recognition (red, green, blue, etc.)
 * - LED feedback matching detected color
 * - Color-triggered actions
 *
 * Hardware: Works in simulation mode without physical hardware
 *
 * @author Boris Mach
 */

#include <NIKLABOTENTA.h>

// Create robot instance
NIKLABOTENTA robot;

// Color detection state
struct DetectedColor {
    uint8_t r, g, b;
    String name;
    unsigned long detected_time;
};

DetectedColor current_color;

// Color-based behaviors
void react_to_color(uint8_t r, uint8_t g, uint8_t b) {
    // Set LED to match detected color
    robot.set_led_color(r, g, b);

    // Determine dominant color
    String color_name = get_color_name(r, g, b);

    // Color-specific reactions
    if (color_name == "red") {
        // Red = Stop
        robot.stop();
        Serial.println("   → Action: STOP (red detected)");

    } else if (color_name == "green") {
        // Green = Go forward
        robot.drive(10.0f, 0.0f);
        Serial.println("   → Action: MOVE FORWARD (green detected)");

    } else if (color_name == "blue") {
        // Blue = Turn right
        robot.drive(0.0f, 45.0f);
        Serial.println("   → Action: TURN RIGHT (blue detected)");

    } else if (color_name == "yellow") {
        // Yellow = Turn left
        robot.drive(0.0f, -45.0f);
        Serial.println("   → Action: TURN LEFT (yellow detected)");

    } else {
        // Unknown color = slow stop
        robot.drive(5.0f, 0.0f);
        Serial.println("   → Action: SLOW FORWARD (unknown color)");
    }
}

// Simple color name from RGB
String get_color_name(uint8_t r, uint8_t g, uint8_t b) {
    // Find dominant channel
    uint8_t max_val = max(max(r, g), b);
    uint8_t min_val = min(min(r, g), b);

    // Low saturation = gray/white/black
    if (max_val - min_val < 50) {
        if (max_val < 80) return "black";
        if (max_val > 200) return "white";
        return "gray";
    }

    // Determine dominant color
    if (r > g && r > b) {
        if (g > b * 1.5f) return "yellow";
        if (b > g * 1.5f) return "magenta";
        return "red";
    } else if (g > r && g > b) {
        if (r > b * 1.5f) return "yellow";
        if (b > r * 1.5f) return "cyan";
        return "green";
    } else {
        if (r > g * 1.5f) return "magenta";
        if (g > r * 1.5f) return "cyan";
        return "blue";
    }
}

void setup() {
    // Initialize Serial for debug output
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        ; // Wait for Serial (max 3 seconds)
    }

    Serial.println("====================================");
    Serial.println("  COLOR DETECTION DEMO");
    Serial.println("  Phase 4: Nicla Vision Camera");
    Serial.println("====================================");

    // Enable debug output
    robot.set_debug(true);

    // Initialize robot
    Serial.println("\nInitializing robot...");
    if (robot.begin()) {
        Serial.println("Robot initialized successfully!");
    } else {
        Serial.println("ERROR: Robot initialization failed!");
        while (1) {
            delay(1000);
        }
    }

    if (robot.is_simulation_mode()) {
        Serial.println("\n⚠ SIMULATION MODE");
        Serial.println("Detecting simulated red color");
        Serial.println("(in hardware, actual camera colors)\n");
    }

    Serial.println("\n--- Starting color detection ---\n");
    Serial.println("Color-based behaviors:");
    Serial.println("🔴 RED    → Stop");
    Serial.println("🟢 GREEN  → Move forward");
    Serial.println("🔵 BLUE   → Turn right");
    Serial.println("🟡 YELLOW → Turn left");
    Serial.println("⚪ OTHER  → Slow forward\n");

    delay(2000);

    // Initialize color
    current_color.r = 0;
    current_color.g = 0;
    current_color.b = 0;
    current_color.name = "none";
    current_color.detected_time = 0;
}

void loop() {
    static unsigned long last_detection = 0;
    unsigned long detection_interval = 200;  // Detect every 200ms

    if (millis() - last_detection >= detection_interval) {
        last_detection = millis();

        // Detect color
        uint8_t r, g, b;
        bool color_found = robot.detect_color(r, g, b);

        if (color_found) {
            // Check if color changed significantly
            int color_diff = abs(r - current_color.r) +
                           abs(g - current_color.g) +
                           abs(b - current_color.b);

            if (color_diff > 30) {  // Significant color change
                // Update current color
                current_color.r = r;
                current_color.g = g;
                current_color.b = b;
                current_color.name = get_color_name(r, g, b);
                current_color.detected_time = millis();

                // Print detection
                Serial.println("====================================");
                Serial.println("COLOR DETECTED");
                Serial.println("====================================");

                Serial.print("RGB: (");
                Serial.print(r);
                Serial.print(", ");
                Serial.print(g);
                Serial.print(", ");
                Serial.print(b);
                Serial.println(")");

                // Visual color bar
                Serial.println("\nColor preview:");
                Serial.print("R: ");
                print_bar(r, 255, 20);
                Serial.print("G: ");
                print_bar(g, 255, 20);
                Serial.print("B: ");
                print_bar(b, 255, 20);

                Serial.print("\nColor name: ");
                Serial.println(current_color.name);

                // React to color
                react_to_color(r, g, b);

                Serial.println("====================================\n");
            }

        } else {
            // No color detected
            robot.stop();
            robot.led_off();

            static unsigned long last_no_color_msg = 0;
            if (millis() - last_no_color_msg > 2000) {
                Serial.println("No color detected...");
                last_no_color_msg = millis();
            }
        }
    }

    delay(10);
}

// Helper function to print a visual bar
void print_bar(uint8_t value, uint8_t max_val, int width) {
    int filled = map(value, 0, max_val, 0, width);

    Serial.print("[");
    for (int i = 0; i < width; i++) {
        if (i < filled) {
            Serial.print("█");
        } else {
            Serial.print(" ");
        }
    }
    Serial.print("] ");
    Serial.println(value);
}
