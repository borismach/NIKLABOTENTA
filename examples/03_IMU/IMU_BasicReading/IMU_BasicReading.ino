/**
 * @file IMU_BasicReading.ino
 * @brief Demonstrates BNO055 9-axis IMU reading capabilities
 *
 * This example shows:
 * - Reading orientation (roll, pitch, yaw)
 * - Getting quaternion data (no gimbal lock)
 * - Reading raw sensor data (accel, gyro, mag)
 * - Monitoring calibration status
 * - Using sensor fusion output
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
    Serial.println("  BNO055 IMU - Basic Reading");
    Serial.println("  Phase 3: 9-axis Sensor Fusion");
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

    Serial.println("\n--- Starting IMU reading demo ---\n");
    delay(2000);
}

void loop() {
    static unsigned long last_print = 0;
    unsigned long print_interval = 500;  // Print every 500ms

    if (millis() - last_print >= print_interval) {
        last_print = millis();

        Serial.println("====================================");
        Serial.println("IMU SENSOR READINGS");
        Serial.println("====================================\n");

        // ========================================
        // 1. ORIENTATION (EULER ANGLES)
        // ========================================
        Serial.println("1. Orientation (Euler Angles):");

        float roll = robot.get_roll();
        float pitch = robot.get_pitch();
        float yaw = robot.get_yaw();

        Serial.print("   Roll:  ");
        Serial.print(roll, 2);
        Serial.println(" degrees");

        Serial.print("   Pitch: ");
        Serial.print(pitch, 2);
        Serial.println(" degrees");

        Serial.print("   Yaw:   ");
        Serial.print(yaw, 2);
        Serial.println(" degrees (heading)");

        Serial.println();

        // ========================================
        // 2. QUATERNION (NO GIMBAL LOCK)
        // ========================================
        Serial.println("2. Quaternion (fusion output):");

        float qw, qx, qy, qz;
        robot.get_quaternion(qw, qx, qy, qz);

        Serial.print("   W: ");
        Serial.print(qw, 4);
        Serial.print("  X: ");
        Serial.print(qx, 4);
        Serial.print("  Y: ");
        Serial.print(qy, 4);
        Serial.print("  Z: ");
        Serial.println(qz, 4);

        // Calculate quaternion magnitude (should be ~1.0)
        float mag = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        Serial.print("   Magnitude: ");
        Serial.print(mag, 4);
        Serial.println(" (should be ~1.0)");

        Serial.println();

        // ========================================
        // 3. RAW ACCELEROMETER & GYROSCOPE
        // ========================================
        Serial.println("3. Raw IMU Data:");

        float ax, ay, az, gx, gy, gz;
        robot.get_imu(ax, ay, az, gx, gy, gz);

        Serial.println("   Accelerometer (m/s²):");
        Serial.print("     X: ");
        Serial.print(ax, 3);
        Serial.print("  Y: ");
        Serial.print(ay, 3);
        Serial.print("  Z: ");
        Serial.println(az, 3);

        Serial.println("   Gyroscope (rad/s):");
        Serial.print("     X: ");
        Serial.print(gx, 3);
        Serial.print("  Y: ");
        Serial.print(gy, 3);
        Serial.print("  Z: ");
        Serial.println(gz, 3);

        Serial.println();

        // ========================================
        // 4. MAGNETOMETER
        // ========================================
        Serial.println("4. Magnetometer (µT):");

        float mx, my, mz;
        robot.get_magnetometer(mx, my, mz);

        Serial.print("   X: ");
        Serial.print(mx, 2);
        Serial.print("  Y: ");
        Serial.print(my, 2);
        Serial.print("  Z: ");
        Serial.println(mz, 2);

        // Calculate total field strength
        float field = sqrt(mx*mx + my*my + mz*mz);
        Serial.print("   Field strength: ");
        Serial.print(field, 2);
        Serial.println(" µT");

        Serial.println();

        // ========================================
        // 5. CALIBRATION STATUS
        // ========================================
        Serial.println("5. Calibration Status:");

        uint8_t cal_level = robot.get_imu_calibration_status();
        Serial.print("   Overall: ");
        Serial.print(cal_level);
        Serial.println("/3");

        if (cal_level == 3) {
            Serial.println("   ✓ Fully calibrated!");
        } else if (cal_level == 2) {
            Serial.println("   ⚠ Partially calibrated");
        } else {
            Serial.println("   ✗ Not calibrated");
        }

        Serial.println("\n====================================\n");

        // Add some visual feedback
        delay(10);
    }

    // Small delay to prevent overwhelming the serial port
    delay(10);
}
