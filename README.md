# Tactile Sensor ROS 2 Bridge: Arduino to ROS 2 Jazzy with Python🖐️📡

This project provides a robust Python bridge to connect an **Arduino-based tactile sensor** to **ROS 2 Jazzy**. It leverages a Python node for serial communication, enabling seamless data flow from your physical sensor to your ROS 2 ecosystem.

---

## 🚀 Getting Started

Follow these steps to set up your tactile sensor bridge.

---

### 🔌 Step 1: Prepare the Arduino

First, you'll need to flash your Arduino with the appropriate firmware.

1.  **Upload the Sketch:**
    * Navigate to the `arduino/` directory in this repository (assuming you'll add an `arduino` folder containing your `.ino` sketch here).
    * Open the provided `.ino` sketch in the Arduino IDE.
    * Ensure your Arduino board and port are correctly selected.
    * **Upload** the sketch to your Arduino board. This sketch is responsible for reading sensor data and sending it over the serial port.

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

### 📦 Step 3: Create ROS 2 Python Package

Now, let's set up your ROS 2 workspace and package.

1.  **Create and navigate to your ROS 2 workspace:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2.  **Create the Python package:**
    ```bash
    ros2 pkg create --build-type ament_python --node-name tactile_publisher tactile_sensor_pkg
    ```
    *This creates a package named `tactile_sensor_pkg` with an executable node called `tactile_publisher`.*

---

### ✍️ Step 4: Write the ROS 2 Node

Time to write the Python script that reads from the serial port and publishes data to ROS 2.

1.  **Navigate to your package's Python directory:**
    ```bash
    cd ~/ros2_ws/src/tactile_sensor_pkg/tactile_sensor_pkg
    ```

2.  **Create the Python publisher script:**
    ```bash
    nano tactile_publisher.py
    ```
    *Then, **paste your Python script here**. This script will handle the serial communication with your Arduino and publish the sensor data to a ROS 2 topic (e.g., as `std_msgs/msg/Int32` or `Float32`).*

    *To save and exit nano: Press `CTRL + O`, then `ENTER`, then `CTRL + X`.*

---

### ⚙️ Step 5: Configure Package Files

We need to tell ROS 2 how to find and run your new Python node, and add necessary dependencies.

1.  **Install `python3-serial` module:**
    ```bash
    sudo apt update
    sudo apt install python3-serial
    ```

2.  **Update `setup.py`** (located at `~/ros2_ws/src/tactile_sensor_pkg/setup.py`):
    *Open this file and add the `entry_points` dictionary. This tells ROS 2 about your executable node.*
    ```python
    from setuptools import find_packages, setup

    package_name = 'tactile_sensor_pkg'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='your_name', # <--- Update this!
        maintainer_email='your_email@example.com', # <--- Update this!
        description='ROS 2 package for tactile sensor bridge',
        license='Apache-2.0', # Or MIT, matching your LICENSE file
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'tactile_publisher = tactile_sensor_pkg.tactile_publisher:main',
            ],
        },
    )
    ```

3.  **Update `package.xml`** (located at `~/ros2_ws/src/tactile_sensor_pkg/package.xml`):
    *Add the necessary runtime dependencies for your node.*
    ```xml
    <?xml version="1.0"?>
    <?xml-model href="[http://download.ros.org/schema/package_format3.xsd](http://download.ros.org/schema/package_format3.xsd)" schematypens="[http://www.w3.org/2001/XMLSchema](http://www.w3.org/2001/XMLSchema)"?>
    <package format="3">
      <name>tactile_sensor_pkg</name>
      <version>0.0.0</version>
      <description>ROS 2 package for tactile sensor bridge</description>
      <maintainer email="your_email@example.com">your_name</maintainer> <license>Apache-2.0</license> <depend>rclpy</depend>
      <depend>std_msgs</depend> <depend>pyserial</depend> <test_depend>ament_copyright</test_depend>
      <test_depend>ament_flake8</test_depend>
      <test_depend>ament_pep257</test_depend>
      <test_depend>python3-pytest</test_depend>

      <export>
        <build_type>ament_python</build_type>
      </export>
    </package>
    ```
    *Make sure to update the `maintainer` tags with your information.*

---

### 🔨 Step 6: Build, Source, and Run

Finally, build your ROS 2 package and run the node!

1.  **Navigate to the root of your workspace:**
    ```bash
    cd ~/ros2_ws
    ```

2.  **Build your package:**
    ```bash
    colcon build --packages-select tactile_sensor_pkg
    ```
    *This compiles your Python package.*

3.  **Source your workspace:**
    ```bash
    source install/setup.bash
    ```
    *This makes your new package and its nodes available to ROS 2.*

4.  **Run your publisher node:**
    ```bash
    ros2 run tactile_sensor_pkg tactile_publisher
    ```
    *You should now see output from your node (e.g., publishing messages or serial read errors if not connected).*

    *To verify messages, open another terminal (and `source install/setup.bash` again) and run:*
    ```bash
    ros2 topic echo /tactile_data # Adjust topic name if different
    ```

---

### 🔁 Rebuild After Code Changes

Whenever you modify your `tactile_publisher.py` (or any other Python source file in the package), you need to re-source your workspace for the changes to take effect. If you modify `setup.py` or `package.xml`, you'll need to rebuild.

```bash
cd ~/ros2_ws
colcon build --packages-select tactile_sensor_pkg # Only needed if setup.py/package.xml changed
source install/setup.bash
ros2 run tactile_sensor_pkg tactile_publisher


### 📁 Repository Structure
.
├── arduino/
│   └── YOUR_ARDUINO_SKETCH.ino  # Add your Arduino sketch here
├── ros2_ws/
│   ├── src/
│   │   └── tactile_sensor_pkg/
│   │       ├── tactile_sensor_pkg/
│   │       │   └── tactile_publisher.py
│   │       ├── package.xml
│   │       └── setup.py
│   ├── .gitignore              # Recommended for ROS 2 repos
│   └── README.md               # This file!
└── LICENSE                     # Your project's license file



### 📌 Important Notes
Device Access Errors: If you're running into device access errors, double-check that /dev/ttyACM0 (or ttyUSB0) is accessible and that your user is indeed in the dialout group (you must restart your WSL terminal after adding yourself).

USB Device Disconnection: You may need to reattach the USB device using usbipd attach if it becomes disconnected (e.g., after a WSL restart or unplugging the Arduino).

Baud Rate: Ensure the baud rate in your Arduino sketch matches the baud rate configured in your tactile_publisher.py script.

Message Type: The example tactile_publisher.py uses std_msgs/msg/Int32. Adjust this (Float32, String, etc.) and the parsing logic in your Python script to match the data type your Arduino sends.

---

✅ .gitignore for your ROS 2 Repository
Create a .gitignore file at the root of your ros2_ws/ directory to prevent unnecessary files from being tracked by Git. This keeps your repository clean and manageable.

# ROS 2 build files
build/
install/
log/

# Python cache
__pycache__/
*.pyc
*.pyo

# VSCode settings
.vscode/

---

🧾 License
This project is licensed under the MIT License. See the LICENSE file in the root of the repository for full details.
---