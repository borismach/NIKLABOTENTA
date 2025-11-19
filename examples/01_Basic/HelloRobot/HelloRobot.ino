/**
 * @file HelloRobot.ino
 * @brief Basic example demonstrating NIKLABOTENTA library usage
 *
 * This example shows:
 * - Library initialization
 * - Basic movement commands
 * - IMU reading
 * - LED control
 * - Debug output
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
    Serial.println("NIKLABOTENTA - Hello Robot Demo");
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

    Serial.println("\n--- Starting demo sequence ---\n");
    delay(2000);
}

void loop() {
    // Demo sequence runs once
    static bool demo_complete = false;

    if (!demo_complete) {
        // 1. LED Test
        Serial.println("1. LED Test - Cycling colors");
        robot.set_led_color(255, 0, 0);  // Red
        delay(500);
        robot.set_led_color(0, 255, 0);  // Green
        delay(500);
        robot.set_led_color(0, 0, 255);  // Blue
        delay(500);
        robot.led_off();
        Serial.println("   LED test complete\n");

        // 2. Movement Test
        Serial.println("2. Movement Test");
        Serial.println("   Moving forward 20 cm...");
        robot.move(20);  // Move forward 20 cm
        delay(1000);

        Serial.println("   Rotating 90 degrees clockwise...");
        robot.rotate(90);  // Turn right
        delay(1000);

        Serial.println("   Moving backward 10 cm...");
        robot.move(-10);  // Move backward
        delay(1000);

        Serial.println("   Rotating 180 degrees...");
        robot.rotate(180);  // Turn around
        delay(1000);
        Serial.println("   Movement test complete\n");

        // 3. IMU Test
        Serial.println("3. IMU Test - Reading orientation");
        float heading = robot.get_orientation();
        float roll = robot.get_roll();
        float pitch = robot.get_pitch();

        Serial.print("   Heading: ");
        Serial.print(heading, 2);
        Serial.println(" deg");

        Serial.print("   Roll: ");
        Serial.print(roll, 2);
        Serial.println(" deg");

        Serial.print("   Pitch: ");
        Serial.print(pitch, 2);
        Serial.println(" deg");

        // Read raw IMU data
        float ax, ay, az, gx, gy, gz;
        robot.get_imu(ax, ay, az, gx, gy, gz);

        Serial.println("\n   Accelerometer (m/s²):");
        Serial.print("     X: ");
        Serial.print(ax, 3);
        Serial.print(" Y: ");
        Serial.print(ay, 3);
        Serial.print(" Z: ");
        Serial.println(az, 3);

        Serial.println("\n   Gyroscope (rad/s):");
        Serial.print("     X: ");
        Serial.print(gx, 3);
        Serial.print(" Y: ");
        Serial.print(gy, 3);
        Serial.print(" Z: ");
        Serial.println(gz, 3);

        // Read quaternion (extended feature)
        float qw, qx, qy, qz;
        robot.get_quaternion(qw, qx, qy, qz);
        Serial.println("\n   Quaternion:");
        Serial.print("     W: ");
        Serial.print(qw, 3);
        Serial.print(" X: ");
        Serial.print(qx, 3);
        Serial.print(" Y: ");
        Serial.print(qy, 3);
        Serial.print(" Z: ");
        Serial.println(qz, 3);

        Serial.println("   IMU test complete\n");

        // 4. Servo Test
        Serial.println("4. Servo Test");
        robot.set_servo_position(0, 0);    // Servo 0 to 0°
        delay(500);
        robot.set_servo_position(0, 90);   // Servo 0 to 90°
        delay(500);
        robot.set_servo_position(0, 180);  // Servo 0 to 180°
        delay(500);
        robot.set_servo_position(0, 90);   // Back to center
        Serial.println("   Servo test complete\n");

        // 5. Sensor Test
        Serial.println("5. Sensor Test");
        float distance = robot.get_distance();
        Serial.print("   Distance: ");
        Serial.print(distance, 1);
        Serial.println(" cm");

        uint8_t r, g, b;
        if (robot.detect_color(r, g, b)) {
            Serial.print("   Detected color - R: ");
            Serial.print(r);
            Serial.print(" G: ");
            Serial.print(g);
            Serial.print(" B: ");
            Serial.println(b);
        }
        Serial.println("   Sensor test complete\n");

        // 6. Battery Status
        Serial.println("6. Battery Status");
        float voltage = robot.get_battery_voltage();
        Serial.print("   Battery voltage: ");
        Serial.print(voltage, 2);
        Serial.println(" V");
        Serial.print("   Charging: ");
        Serial.println(robot.is_charging() ? "Yes" : "No");
        Serial.println("   Battery test complete\n");

        // Demo complete
        Serial.println("=================================");
        Serial.println("Demo sequence complete!");
        Serial.println("=================================");
        Serial.println("\nRobot is now idle.");
        Serial.println("Reset to run demo again.\n");

        demo_complete = true;
    }

    // After demo, just blink LED slowly
    static unsigned long last_blink = 0;
    static bool led_state = false;

    if (millis() - last_blink > 2000) {
        if (led_state) {
            robot.set_led_color(0, 50, 0);  // Dim green
        } else {
            robot.led_off();
        }
        led_state = !led_state;
        last_blink = millis();
    }

    delay(100);
}
