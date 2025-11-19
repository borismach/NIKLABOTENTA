# NIKLABOTENTA

An Arduino library that replicates the Arduino Alvik robot API using alternative hardware: **Nicla Vision**, **Portenta H7**, and **BNO055 STEMMA IMU**.

## Why NIKLABOTENTA?

Build your own Alvik-compatible robot with more powerful hardware:
- 🎥 **Computer Vision** - Nicla Vision camera for object detection and tracking
- 🧭 **9-axis IMU** - BNO055 with sensor fusion for precise orientation
- ⚡ **Dual-core power** - Portenta H7 for parallel processing
- 🔌 **Familiar API** - Same commands as Arduino Alvik

## Features

### Phase 2: Motor Control System ✅
- **Differential Drive** - Full 2-wheel robot kinematics
- **Encoder Integration** - Interrupt-based tick counting and odometry
- **PID Speed Control** - Precise velocity regulation with anti-windup
- **Simulation Mode** - Test without physical hardware
- Alvik-compatible movement commands (`move()`, `rotate()`, `drive()`)

### Phase 3: BNO055 9-Axis IMU ✅
- **Sensor Fusion** - Accelerometer + Gyroscope + Magnetometer integration
- **Quaternion Output** - Gimbal lock-free orientation representation
- **Euler Angles** - Roll, pitch, yaw in degrees
- **Automatic Calibration** - Interactive calibration procedure
- **Raw Sensor Access** - Direct access to all 9-axis data
- **Magnetometer** - Absolute heading/compass functionality

### Planned Features
- Camera-based line following and object detection (Phase 4)
- Servo control and LED management
- Extensible architecture

## Hardware Requirements

- Arduino Portenta H7
- Arduino Nicla Vision
- Adafruit BNO055 STEMMA
- DC motors with encoders
- (Optional) Servo motors

## Quick Start

### Basic Movement
```cpp
#include <NIKLABOTENTA.h>

NIKLABOTENTA robot;

void setup() {
    robot.begin();
}

void loop() {
    robot.move(10);      // move forward 10cm
    robot.rotate(90);    // turn 90 degrees
    delay(1000);
}
```

### Advanced Motor Control
```cpp
// Differential drive with speed and rotation
robot.drive(15.0, 30.0);  // 15 cm/s forward, 30 deg/s right turn

// Get wheel velocities
float left, right;
robot.get_speed(left, right);
```

### IMU Sensor Reading
```cpp
// Get orientation (Euler angles)
float roll = robot.get_roll();      // tilt left/right
float pitch = robot.get_pitch();    // tilt forward/back
float heading = robot.get_yaw();    // compass direction (0-360°)

// Get quaternion (no gimbal lock)
float qw, qx, qy, qz;
robot.get_quaternion(qw, qx, qy, qz);

// Get raw sensor data
float ax, ay, az, gx, gy, gz;
robot.get_imu(ax, ay, az, gx, gy, gz);  // accel + gyro
```

See [examples/](examples/) for more demonstrations.

## Documentation

- [API Reference](docs/API.md)
- [Hardware Setup](docs/HARDWARE.md)
- [Examples](examples/)

## Contributing

Contributions are welcome! Please read [CONTRIBUTING.md](CONTRIBUTING.md) before submitting pull requests.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Disclaimer

NIKLABOTENTA is an independent project and is not affiliated with or endorsed by Arduino or Adafruit.

Arduino®, Alvik®, Portenta®, and Nicla® are trademarks of Arduino SA.
BNO055 and STEMMA are products of Adafruit Industries.
