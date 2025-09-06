# Tactile Sensor Package

This ROS2 package provides functionality for reading data from a 1x4 Velostat-based tactile sensor array connected via Arduino and visualizing the data with threshold-based binary touch detection.

## Features

- **Raw Data Publishing**: Publishes raw analog values from all 4 sensors
- **Binary Touch Detection**: Converts raw values to binary touch states using configurable threshold
- **Real-time Visualization**: Live plotting of raw values, binary states, and overall touch detection
- **Configurable Parameters**: Adjustable serial port, baud rate, and touch threshold

## Topics

The package publishes to the following topics:

- `/tactile_raw_values` (std_msgs/Int32MultiArray): Raw analog values from all 4 sensors
- `/tactile_binary_array` (std_msgs/Int32MultiArray): Binary touch states (0/1) for each sensor
- `/tactile_touch_detected` (std_msgs/Bool): Overall touch detection (True if any sensor is touched)

## 📁 Repository Structure
```
.
├── arduino_code/
│   ├── README.md
│   └── YOUR_ARDUINO_SKETCH.ino
├── build/
├── install/
├── log/
├── src/
│   └── tactile_sensor_pkg/
│       ├── tactile_sensor_pkg/
│       │   └── tactile_publisher.py
│       ├── package.xml
│       ├── README.md
│       └── setup.py
├── .gitignore
├── LICENSE
└── README.md
```

## Installation

1. Install the required Python dependencies:
```bash
pip install -r requirements.txt
```

2. Build the package:
```bash
colcon build --packages-select tactile_sensor_pkg
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Using the Launch File (Recommended)

Run both the publisher and visualizer together:
```bash
ros2 launch tactile_sensor_pkg tactile_sensor.launch.py
```

You can customize parameters:
```bash
ros2 launch tactile_sensor_pkg tactile_sensor.launch.py serial_port:=/dev/ttyUSB0 touch_threshold:=700
```

### Running Nodes Individually

1. Start the tactile publisher:
```bash
ros2 run tactile_sensor_pkg tactile_publisher --ros-args -p serial_port:=/dev/ttyACM0 -p touch_threshold:=800
```

2. Start the visualizer (in a separate terminal):
```bash
ros2 run tactile_sensor_pkg tactile_visualizer
```

### Monitoring Topics

You can monitor the published data using:
```bash
# Raw values
ros2 topic echo /tactile_raw_values

# Binary array
ros2 topic echo /tactile_binary_array

# Touch detection
ros2 topic echo /tactile_touch_detected
```

## Configuration

### Parameters

- `serial_port` (string): Serial port for Arduino connection (default: `/dev/ttyACM0`)
- `baud_rate` (int): Baud rate for serial communication (default: `115200`)
- `touch_threshold` (int): Threshold for binary touch detection, range 0-1023 (default: `650`). Touch is detected when sensor value goes BELOW this threshold (inverted behavior due to velostat resistance decreasing on touch)

### Arduino Code

The Arduino should send comma-separated values in the format:
```
value1,value2,value3,value4
```

Example Arduino code is provided in the comments of the main publisher file.

## Visualization

The visualizer provides three real-time plots:

1. **Raw Sensor Values**: Shows the analog values from all 4 sensors over time
2. **Binary Touch States**: Shows the binary touch state (0/1) for each sensor
3. **Overall Touch Detection**: Shows whether any sensor is currently being touched

## Troubleshooting

1. **Serial Port Issues**: Make sure the Arduino is connected and the serial port is correct
2. **Permission Issues**: You may need to add your user to the `dialout` group:
   ```bash
   sudo usermod -a -G dialout $USER
   ```
3. **Threshold Adjustment**: If touch detection is too sensitive or not sensitive enough, adjust the `touch_threshold` parameter

## Dependencies

- ROS2 (tested with Jazzy)
- Python 3+
- matplotlib
- numpy
- pyserial
