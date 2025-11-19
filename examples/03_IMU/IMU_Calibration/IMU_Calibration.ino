/**
 * @file IMU_Calibration.ino
 * @brief Interactive IMU calibration demonstration
 *
 * This example shows:
 * - Interactive calibration procedure
 * - Real-time calibration status monitoring
 * - Visual calibration progress indicators
 * - Post-calibration orientation testing
 *
 * Calibration Procedure (for real hardware):
 * 1. GYROSCOPE: Keep sensor stationary
 * 2. ACCELEROMETER: Place sensor in 6 different orientations
 * 3. MAGNETOMETER: Move sensor in figure-8 pattern
 * 4. SYSTEM: Wait for all sensors to calibrate
 *
 * Hardware: Works in simulation mode (auto-calibrates)
 *
 * @author Boris Mach
 */

#include <NIKLABOTENTA.h>

// Create robot instance
NIKLABOTENTA robot;

// Calibration state
enum CalState {
    CAL_INIT,
    CAL_RUNNING,
    CAL_COMPLETE,
    CAL_TESTING
};

CalState state = CAL_INIT;

void setup() {
    // Initialize Serial for debug output
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        ; // Wait for Serial (max 3 seconds)
    }

    Serial.println("=================================");
    Serial.println("  BNO055 IMU - Calibration");
    Serial.println("  Phase 3: Interactive Procedure");
    Serial.println("=================================");

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
        Serial.println("Auto-calibrating for demonstration...\n");
    } else {
        Serial.println("\n🔧 HARDWARE MODE");
        Serial.println("Follow calibration instructions below:\n");
    }

    delay(1000);
}

void loop() {
    switch (state) {
        case CAL_INIT:
            showInstructions();
            state = CAL_RUNNING;
            break;

        case CAL_RUNNING:
            runCalibration();
            break;

        case CAL_COMPLETE:
            showCalibrationComplete();
            state = CAL_TESTING;
            break;

        case CAL_TESTING:
            testOrientation();
            break;
    }
}

void showInstructions() {
    Serial.println("====================================");
    Serial.println("CALIBRATION INSTRUCTIONS");
    Serial.println("====================================\n");

    Serial.println("The BNO055 needs calibration for:");
    Serial.println();

    Serial.println("1. GYROSCOPE (measures rotation)");
    Serial.println("   → Keep sensor completely still");
    Serial.println("   → Takes ~5 seconds");
    Serial.println();

    Serial.println("2. ACCELEROMETER (measures tilt)");
    Serial.println("   → Slowly move through 6 orientations:");
    Serial.println("     - Flat on table");
    Serial.println("     - Upside down");
    Serial.println("     - Each of 4 sides");
    Serial.println("   → Hold each position for 2-3 seconds");
    Serial.println();

    Serial.println("3. MAGNETOMETER (measures heading)");
    Serial.println("   → Move sensor in figure-8 pattern");
    Serial.println("   → Rotate through all 3 axes");
    Serial.println("   → Continue for 20-30 seconds");
    Serial.println();

    Serial.println("4. SYSTEM (overall fusion)");
    Serial.println("   → Automatically calibrates after sensors");
    Serial.println();

    Serial.println("====================================");
    Serial.println("Press any key to start calibration...");
    Serial.println("====================================\n");

    // Wait for user input
    while (!Serial.available()) {
        delay(100);
    }

    // Clear input buffer
    while (Serial.available()) {
        Serial.read();
    }

    Serial.println("\n🚀 Starting calibration...\n");
    delay(500);
}

void runCalibration() {
    static unsigned long last_update = 0;
    unsigned long update_interval = 500;  // Update every 500ms

    // Start calibration
    bool success = robot.calibrate_imu();

    if (success) {
        state = CAL_COMPLETE;
    } else {
        // Calibration failed or timed out
        Serial.println("\n⚠ Calibration incomplete or failed");
        Serial.println("Current status:");

        uint8_t cal = robot.get_imu_calibration_status();
        Serial.print("Overall calibration: ");
        Serial.print(cal);
        Serial.println("/3");

        Serial.println("\nYou can:");
        Serial.println("1. Continue using partially calibrated data");
        Serial.println("2. Reset and try again (send 'r')");
        Serial.println();

        // Wait for user choice
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'r' || c == 'R') {
                Serial.println("Restarting calibration...\n");
                state = CAL_INIT;
                return;
            }
        }

        // Continue to testing anyway
        state = CAL_TESTING;
    }
}

void showCalibrationComplete() {
    Serial.println("\n====================================");
    Serial.println("✓ CALIBRATION COMPLETE!");
    Serial.println("====================================\n");

    Serial.println("All sensors are fully calibrated.");
    Serial.println("Calibration status: 3/3\n");

    Serial.println("The sensor will now provide:");
    Serial.println("- Accurate absolute orientation");
    Serial.println("- Stable heading (compass)");
    Serial.println("- Precise tilt angles");
    Serial.println();

    Serial.println("Press any key to test orientation...");

    // Wait for user input
    while (!Serial.available()) {
        delay(100);
    }

    // Clear input buffer
    while (Serial.available()) {
        Serial.read();
    }

    Serial.println("\n📊 Starting orientation test...\n");
    delay(500);
}

void testOrientation() {
    static unsigned long last_print = 0;
    unsigned long print_interval = 200;  // Fast updates

    if (millis() - last_print >= print_interval) {
        last_print = millis();

        // Clear screen (ANSI escape code - works on most terminals)
        Serial.print("\033[2J\033[H");

        Serial.println("====================================");
        Serial.println("ORIENTATION TEST (Real-time)");
        Serial.println("====================================\n");

        // Get orientation
        float roll = robot.get_roll();
        float pitch = robot.get_pitch();
        float heading = robot.get_orientation();

        // Display with visual bars
        Serial.println("Roll (tilt left/right):");
        printBar(roll, -180, 180, 40);
        Serial.print("  ");
        Serial.print(roll, 1);
        Serial.println("°\n");

        Serial.println("Pitch (tilt forward/back):");
        printBar(pitch, -90, 90, 40);
        Serial.print("  ");
        Serial.print(pitch, 1);
        Serial.println("°\n");

        Serial.println("Heading (compass direction):");
        printBar(heading, 0, 360, 40);
        Serial.print("  ");
        Serial.print(heading, 1);
        Serial.println("°");

        // Show cardinal direction
        String direction = getCardinalDirection(heading);
        Serial.print("  Direction: ");
        Serial.println(direction);

        Serial.println("\n====================================");
        Serial.println("Move the sensor to see changes");
        Serial.println("Send 'r' to recalibrate");
        Serial.println("====================================\n");

        // Check for recalibration request
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'r' || c == 'R') {
                state = CAL_INIT;
                delay(500);
            }
        }
    }

    delay(10);
}

// Helper function to print a visual bar
void printBar(float value, float min_val, float max_val, int width) {
    // Normalize value to 0-width range
    int pos = map(constrain(value, min_val, max_val), min_val, max_val, 0, width);

    Serial.print("  [");
    for (int i = 0; i < width; i++) {
        if (i == pos) {
            Serial.print("|");
        } else if (i == width / 2) {
            Serial.print("·");
        } else {
            Serial.print(" ");
        }
    }
    Serial.print("]");
}

// Helper function to get cardinal direction from heading
String getCardinalDirection(float heading) {
    if (heading >= 337.5 || heading < 22.5) return "N (North)";
    if (heading >= 22.5 && heading < 67.5) return "NE (Northeast)";
    if (heading >= 67.5 && heading < 112.5) return "E (East)";
    if (heading >= 112.5 && heading < 157.5) return "SE (Southeast)";
    if (heading >= 157.5 && heading < 202.5) return "S (South)";
    if (heading >= 202.5 && heading < 247.5) return "SW (Southwest)";
    if (heading >= 247.5 && heading < 292.5) return "W (West)";
    if (heading >= 292.5 && heading < 337.5) return "NW (Northwest)";
    return "?";
}
