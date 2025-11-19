/**
 * @file LineFollower.ino
 * @brief Basic line following demonstration using Nicla Vision camera
 *
 * This example shows:
 * - Real-time line detection
 * - Proportional steering based on line position
 * - Visual feedback of line tracking
 * - Simulation mode for testing logic
 *
 * Line Following Algorithm:
 * 1. Detect line in camera frame
 * 2. Calculate line position (-1 = left, 0 = center, 1 = right)
 * 3. Apply proportional steering correction
 * 4. Drive forward while steering
 *
 * Hardware: Works in simulation mode without physical hardware
 *
 * @author Boris Mach
 */

#include <NIKLABOTENTA.h>

// Create robot instance
NIKLABOTENTA robot;

// Line following parameters
const float BASE_SPEED = 10.0f;      // Forward speed (cm/s)
const float STEERING_GAIN = 30.0f;   // Steering sensitivity (deg/s per unit position)
const float LOST_LINE_TIMEOUT = 2000; // ms before stopping if line lost

// State tracking
unsigned long last_line_seen = 0;
bool line_following_active = false;

void setup() {
    // Initialize Serial for debug output
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        ; // Wait for Serial (max 3 seconds)
    }

    Serial.println("====================================");
    Serial.println("  LINE FOLLOWER DEMO");
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
        Serial.println("Line position will vary randomly");
        Serial.println("(demonstrates steering logic)\n");
    }

    Serial.println("\n--- Starting line follower ---\n");
    Serial.println("Algorithm:");
    Serial.println("- Detect line using camera");
    Serial.println("- Calculate steering correction");
    Serial.println("- Drive forward with proportional steering\n");

    delay(2000);

    // Enable line following
    line_following_active = true;
    Serial.println("Line following ACTIVE\n");
}

void loop() {
    static unsigned long last_print = 0;
    unsigned long print_interval = 200;  // Status update every 200ms

    if (line_following_active) {
        // Detect line
        bool line_detected = robot.detect_line();

        if (line_detected) {
            // Update last seen time
            last_line_seen = millis();

            // Get line position (-1.0 to 1.0)
            float line_pos = robot.get_line_position();

            // Calculate steering correction (proportional control)
            // Positive position (right) → turn right (positive rotation)
            // Negative position (left) → turn left (negative rotation)
            float steering = -line_pos * STEERING_GAIN;

            // Drive with steering correction
            robot.drive(BASE_SPEED, steering);

            // Status output
            if (millis() - last_print >= print_interval) {
                last_print = millis();

                Serial.print("Line detected | Position: ");
                Serial.print(line_pos, 2);

                // Visual position indicator
                Serial.print(" [");
                int bar_pos = map(line_pos * 100, -100, 100, 0, 20);
                for (int i = 0; i < 20; i++) {
                    if (i == bar_pos) {
                        Serial.print("|");
                    } else if (i == 10) {
                        Serial.print("·");
                    } else {
                        Serial.print(" ");
                    }
                }
                Serial.print("] | Steering: ");
                Serial.print(steering, 1);
                Serial.println(" deg/s");
            }

        } else {
            // Line not detected
            unsigned long time_since_line = millis() - last_line_seen;

            if (time_since_line < LOST_LINE_TIMEOUT) {
                // Recently lost line - slow down and search
                robot.drive(BASE_SPEED * 0.5f, 0.0f);

                if (millis() - last_print >= print_interval) {
                    last_print = millis();
                    Serial.print("Line LOST | Searching... (");
                    Serial.print((LOST_LINE_TIMEOUT - time_since_line) / 1000.0f, 1);
                    Serial.println("s)");
                }

            } else {
                // Line lost for too long - stop
                robot.stop();
                line_following_active = false;

                Serial.println("\n====================================");
                Serial.println("Line following STOPPED");
                Serial.println("Reason: Line lost for too long");
                Serial.println("====================================\n");

                Serial.println("Send any character to restart...");
            }
        }

    } else {
        // Waiting to restart
        if (Serial.available()) {
            // Clear input
            while (Serial.available()) {
                Serial.read();
            }

            // Restart line following
            Serial.println("\nRestarting line follower...\n");
            line_following_active = true;
            last_line_seen = millis();
        }

        // Blink LED to indicate waiting
        static unsigned long last_blink = 0;
        static bool led_state = false;

        if (millis() - last_blink > 1000) {
            if (led_state) {
                robot.set_led_color(0, 0, 100);  // Blue
            } else {
                robot.led_off();
            }
            led_state = !led_state;
            last_blink = millis();
        }
    }

    delay(10);
}
