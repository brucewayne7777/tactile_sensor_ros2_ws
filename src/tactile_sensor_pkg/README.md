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

## 🚀 Getting Started: Full Setup Guide

Follow these steps to set up the hardware and software bridge.

### 🔌 Step 1: Prepare the Arduino

First, you'll need to flash your Arduino with the appropriate firmware. The Arduino sketch is located in the `arduino_code` directory at the root of this workspace.

1.  **Upload the Sketch:**
    *   Navigate to the `arduino_code/` directory.
    *   Open the `.ino` sketch in the Arduino IDE.
    *   Ensure your Arduino board and port are correctly selected.
    *   **Upload** the sketch to your Arduino board. This sketch is responsible for reading sensor data and sending it over the serial port.

---

### 💻 Step 2: Connect Arduino to ROS 2 in WSL (Ubuntu on Windows)

Since **WSL (Windows Subsystem for Linux) does not have direct access to USB devices**, we'll use `usbipd` to share your Arduino from Windows to WSL.

#### 🧰 Prerequisites (Run in **PowerShell as Administrator**)

1.  **Install `usbipd` utility:**
    ```powershell
    winget install --interactive --exact dorssel.usbipd-win
    ```

2.  **List USB devices and get your Arduino's `BUSID`:**
    ```powershell
    usbipd wsl list
    ```
    *Look for your Arduino (e.g., "Arduino Uno", "CH340", "USB-SERIAL CH340") and note its `BUSID` (e.g., `1-4`).*

3.  **Bind the device to make it shareable:**
    ```powershell
    usbipd bind --busid <BUSID_FROM_ABOVE>
    ```
    *Replace `<BUSID_FROM_ABOVE>` with the actual BUSID you found.*

4.  **Attach the device to your WSL instance:**
    ```powershell
    usbipd attach --wsl --busid <BUSID_FROM_ABOVE>
    ```
    *Again, replace `<BUSID_FROM_ABOVE>` with your device's BUSID.*

#### 🐧 In Ubuntu (WSL)

1.  **Verify connection:**
    ```bash
    ls /dev/ttyACM*
    ```
    *If you see `/dev/ttyACM0` (or similar, like `/dev/ttyUSB0`), your Arduino is successfully connected!*

2.  **Add your user to the `dialout` group for serial port access:**
    ```bash
    sudo usermod -a -G dialout $USER
    ```
    *After running this, **close and reopen your WSL terminal** for the changes to take effect.*

---

### ⚙️ Step 3: Configure the ROS 2 Package

The package is already created, but you may need to install dependencies and ensure it's configured correctly.

1.  **Install Python Dependencies:**
    Make sure you have `pyserial` installed.
    ```bash
    sudo apt update
    sudo apt install python3-serial
    ```

2.  **Check `setup.py`**:
    *The `setup.py` file in this package is already configured with the correct entry points to find the `tactile_publisher` and `tactile_visualizer` nodes.*

3.  **Check `package.xml`**:
    *The `package.xml` file includes the necessary dependencies like `rclpy`, `std_msgs`, and `pyserial`.*

---

### 🔨 Step 4: Build, Source, and Run

Finally, build your ROS 2 package and run the nodes. For detailed run commands, see the [Usage](#usage) section below.

1.  **Navigate to the root of your workspace:**
    ```bash
    cd ~/ros2_ws
    ```

2.  **Build your package:**
    ```bash
    colcon build --packages-select tactile_sensor_pkg
    ```

3.  **Source your workspace:**
    ```bash
    source install/setup.bash
    ```
    *This makes your new package and its nodes available to ROS 2.*

4.  **Run your nodes:**
    *Use the `ros2 launch` or `ros2 run` commands as described in the [Usage](#usage) section.*

---
 
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
