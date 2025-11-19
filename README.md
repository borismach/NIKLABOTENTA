# NIKLABOTENTA

An Arduino library that replicates the Arduino Alvik robot API using alternative hardware: **Nicla Vision**, **Portenta H7**, and **BNO055 STEMMA IMU**.

## Why NIKLABOTENTA?

Build your own Alvik-compatible robot with more powerful hardware:
- 🎥 **Computer Vision** - Nicla Vision camera for object detection and tracking
- 🧭 **9-axis IMU** - BNO055 with sensor fusion for precise orientation
- ⚡ **Dual-core power** - Portenta H7 for parallel processing
- 🔌 **Familiar API** - Same commands as Arduino Alvik

## Features

- Alvik-compatible movement commands (`move()`, `rotate()`, `drive()`)
- Enhanced IMU with quaternions and Euler angles
- Camera-based line following and object detection
- Servo and motor control
- Extensible architecture

## Hardware Requirements

- Arduino Portenta H7
- Arduino Nicla Vision
- Adafruit BNO055 STEMMA
- DC motors with encoders
- (Optional) Servo motors

## Quick Start
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
