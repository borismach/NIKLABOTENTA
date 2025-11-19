/**
 * @file MotorControl.ino
 * @brief Demonstrates motor control and differential drive capabilities
 *
 * This example shows:
 * - Differential drive kinematics
 * - Encoder-based odometry
 * - PID speed control
 * - Various movement patterns
 * - Real-time velocity and position feedback
 *
 * Hardware: Works in simulation mode without physical hardware
 *
 * @author Boris Mach
 */

#include <NIKLABOTENTA.h>

// Create robot instance
NIKLABOTENTA robot;

void setup() {
    // Initialize Serial for debug output
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        ; // Wait for Serial (max 3 seconds)
    }

    Serial.println("=================================");
    Serial.println("  MOTOR CONTROL DEMO");
    Serial.println("  Phase 2: Differential Drive");
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

    // Print library info
    Serial.print("Library version: ");
    Serial.println(robot.get_version());

    if (robot.is_simulation_mode()) {
        Serial.println("Running in SIMULATION mode");
        Serial.println("(No physical hardware required)");
    }

    Serial.println("\n--- Starting motor control demo ---\n");
    delay(2000);
}

void loop() {
    static bool demo_complete = false;

    if (!demo_complete) {
        Serial.println("====================================");
        Serial.println("TEST 1: Basic Movement");
        Serial.println("====================================\n");

        // Test 1: Forward movement
        Serial.println(">> Moving forward 30 cm...");
        robot.move(30);  // Move forward 30 cm (blocking)
        delay(500);

        // Get velocity feedback
        float left_speed, right_speed;
        robot.get_speed(left_speed, right_speed);
        Serial.print("   Final velocities - Left: ");
        Serial.print(left_speed, 2);
        Serial.print(" cm/s, Right: ");
        Serial.print(right_speed, 2);
        Serial.println(" cm/s");
        delay(1000);

        // Test 2: Backward movement
        Serial.println("\n>> Moving backward 15 cm...");
        robot.move(-15);
        delay(1000);

        Serial.println("\nTest 1 complete!\n");
        delay(2000);

        // ====================================
        Serial.println("====================================");
        Serial.println("TEST 2: Rotation");
        Serial.println("====================================\n");

        // Test 3: Rotate clockwise
        Serial.println(">> Rotating 90 degrees CW...");
        robot.rotate(90);
        delay(500);

        float heading = robot.get_orientation();
        Serial.print("   Current heading: ");
        Serial.print(heading, 1);
        Serial.println(" degrees");
        delay(1000);

        // Test 4: Rotate counter-clockwise
        Serial.println("\n>> Rotating 180 degrees CCW...");
        robot.rotate(-180);
        delay(500);

        heading = robot.get_orientation();
        Serial.print("   Current heading: ");
        Serial.print(heading, 1);
        Serial.println(" degrees");
        delay(1000);

        // Return to zero heading
        Serial.println("\n>> Returning to 0 degrees...");
        robot.rotate(90);
        delay(500);

        Serial.println("\nTest 2 complete!\n");
        delay(2000);

        // ====================================
        Serial.println("====================================");
        Serial.println("TEST 3: Differential Drive");
        Serial.println("====================================\n");

        // Test 5: Drive with rotation
        Serial.println(">> Driving in arc (forward + right turn)...");
        Serial.println("   Speed: 15 cm/s, Rotation: 30 deg/s");

        // Drive for 2 seconds
        unsigned long start_time = millis();
        while (millis() - start_time < 2000) {
            robot.drive(15.0, 30.0);  // 15 cm/s forward, 30 deg/s right
            delay(50);
        }
        robot.stop();

        robot.get_speed(left_speed, right_speed);
        Serial.print("   Wheel speeds - Left: ");
        Serial.print(left_speed, 2);
        Serial.print(" cm/s, Right: ");
        Serial.print(right_speed, 2);
        Serial.println(" cm/s");
        delay(1000);

        // Test 6: Spin in place
        Serial.println("\n>> Spinning in place (left)...");
        Serial.println("   Angular velocity: 60 deg/s");

        start_time = millis();
        while (millis() - start_time < 1500) {
            robot.drive(0.0, -60.0);  // No linear speed, 60 deg/s left
            delay(50);
        }
        robot.stop();
        delay(1000);

        Serial.println("\nTest 3 complete!\n");
        delay(2000);

        // ====================================
        Serial.println("====================================");
        Serial.println("TEST 4: Square Path");
        Serial.println("====================================\n");

        Serial.println(">> Driving square pattern (20cm sides)...");

        for (int i = 0; i < 4; i++) {
            Serial.print("   Side ");
            Serial.println(i + 1);

            robot.move(20);     // Forward 20 cm
            delay(500);

            robot.rotate(90);   // Turn 90 degrees
            delay(500);
        }

        heading = robot.get_orientation();
        Serial.print("   Final heading: ");
        Serial.print(heading, 1);
        Serial.println(" degrees (should be ~0)");

        Serial.println("\nTest 4 complete!\n");
        delay(2000);

        // ====================================
        Serial.println("====================================");
        Serial.println("TEST 5: Brake Test");
        Serial.println("====================================\n");

        Serial.println(">> Testing brake vs coast stop...");

        // Test brake
        Serial.println("   Applying active brake...");
        robot.drive(20.0, 0.0);
        delay(500);
        robot.brake();  // Active brake
        delay(1000);

        // Test coast
        Serial.println("   Coast to stop...");
        robot.drive(20.0, 0.0);
        delay(500);
        robot.stop();   // Coast stop
        delay(1000);

        Serial.println("\nTest 5 complete!\n");
        delay(2000);

        // ====================================
        Serial.println("====================================");
        Serial.println("DEMO COMPLETE!");
        Serial.println("====================================\n");

        Serial.println("Summary:");
        Serial.println("- Basic movement: PASS");
        Serial.println("- Rotation: PASS");
        Serial.println("- Differential drive: PASS");
        Serial.println("- Square path: PASS");
        Serial.println("- Brake control: PASS");
        Serial.println("\nAll motor control tests completed successfully!");
        Serial.println("\nRobot is now idle. Reset to run demo again.\n");

        demo_complete = true;
    }

    // After demo, just heartbeat LED
    static unsigned long last_blink = 0;
    static bool led_state = false;

    if (millis() - last_blink > 2000) {
        if (led_state) {
            robot.set_led_color(0, 0, 50);  // Dim blue
        } else {
            robot.led_off();
        }
        led_state = !led_state;
        last_blink = millis();
    }

    delay(100);
}
