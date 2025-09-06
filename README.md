# Tactile Sensor ROS 2 Workspace

This repository contains the ROS 2 workspace for interfacing with a tactile sensor.

## Overview

This project provides a bridge between an Arduino-based tactile sensor and ROS 2 Jazzy. It includes a ROS 2 package for reading sensor data from a serial port and publishing it to a ROS 2 topic.

## Packages

*   [`tactile_sensor_pkg`](./src/tactile_sensor_pkg/README.md): The main ROS 2 package containing the Python node for serial communication and data publishing. For detailed setup and usage instructions, please refer to the package's `README.md`.

## Arduino Firmware

*   [`arduino_code`](./arduino_code/README.md): Contains the Arduino sketch for the tactile sensor. See the README inside for more details.

## License

This project is licensed under the MIT License. See the [`LICENSE`](./LICENSE) file for full details.
