# ROS2 Four Wheel Robot GUI

**ROS2 Four Wheel Robot GUI** is a PyQt5-based graphical interface for controlling and monitoring a ROS 2 robot, including camera viewing, sensor readouts, and control over servos, motors, LEDs, buzzer, and OLED display.

## 📦 Features

- 🕹 Manual control via buttons or keyboard (`WASD` + `SPACE`)
- 📷 Camera feed from `/image_raw`
- 🌡 Real-time sensor readouts:
  - LDR left/right (`/sensors/ldr_left`, `/sensors/ldr_right`)
  - Battery voltage (`/sensors/battery_voltage`)
  - Ultrasonic distance (`/ultrasonic/distance`)
  - Line tracking (`/line_tracking`)
- 🔧 Control components:
  - Wheels via `/cmd_vel`
  - Servos via `/servo/0/angle`, `/servo/1/angle`
  - Buzzer via `/buzzer`
  - LEDs via `/led/set_color`
  - OLED text via `/oled_text`

## 🚀 How to Use

### 1. Install dependencies

```bash
sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport
pip install PyQt5 opencv-python
```

### 2. Build the package

Clone or unzip this into your ROS 2 workspace (`~/ros2_ws/src`):

```bash
cd ~/ros2_ws
colcon build --packages-select kilted_gui
source install/setup.bash
```

### 3. Run the GUI

```bash
ros2 launch kilted_gui kilted_gui.launch.py
```

## 🎮 Controls

Set wheel and motor speed with the sliders. Then press

- `W` – Forward
- `S` – Backward
- `A` – Turn left
- `D` – Turn right
- `Space` – Stop

Or use the on-screen directional buttons.

## 📁 Package Structure

```
kilted_gui/
├── kilted_gui/
│   ├── main.py
│   ├── controller.py
│   ├── camera_widget.py
│   ├── sensor_reader.py
├── launch/
│   └── kilted_gui.launch.py
├── resource/
│   └── kilted_gui
├── setup.py
├── setup.cfg
└── package.xml
```

## 📜 License

MIT License.
