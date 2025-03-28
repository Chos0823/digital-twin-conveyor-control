# Digital Twin Conveyor Control System

This project implements a digital twin control system that integrates Unity, ROS2, and a PyQt5-based GUI to simulate and control an automated object classification and transportation system using a robot and conveyor belt.

---

## 📁 Directory Structure

```
Qt_GUI_with_Conveyor/
├── main.py                    # GUI main script
├── ui_design.ui              # UI design file
├── conveyor_logic.py         # Conveyor control logic
├── config.yaml               # Configuration settings
├── resources/                # Icons and visual assets

digital_twin/
├── models/                   # 3D models for simulation
├── worlds/                   # Gazebo world files
├── launch/                   # ROS2 launch files
├── config/                   # ROS2 parameters
├── CMakeLists.txt, package.xml

turtlebot3_ws/
├── src/
│   ├── turtlebot3/
│   ├── turtlebot3_simulations/
│   └── turtlebot3_msgs/
├── CMakeLists.txt, package.xml
```

---

## ⚙️ Features

- Unity-based digital twin visualization
- ROS2 integration for controlling TurtleBot3 and the conveyor
- PyQt5 GUI for selecting red/blue block counts and goals
- YOLO-based object recognition and sorting
- MoveIt-based robot manipulation

---

## 🚀 How to Run

### 1. TurtleBot3 Setup
```bash
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
```

### 2. Start YOLO-based ArUco Detection (on PC)
```bash
ros2 launch aruco_yolo aruco_yolo.launch.py
```

### 3. Launch MoveIt for Manipulation
```bash
ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py
```

### 4. Run the Manipulator Controller
```bash
ros2 run turtlebot_moveit turtlebot_arm_controller
```

### 5. Start Conveyor Logic
```bash
ros2 run conveyor con
```

### 6. Launch the PyQt5 GUI
```bash
ros2 run conveyor gui
```

### 7. (Optional) Start ROS Bridge Server
```bash
python3 filtered_rosbridge.py
```

### 8. (Optional) Run Unity Digital Twin Environment

### 9. (Optional) Run Final Test Script
```bash
python3 test5.py  # from turtlebot_moveit/scripts/
```

---

## 🪪 License

This project is licensed under the **MIT License** – feel free to use, modify, and distribute.

---

## 🙌 Acknowledgments

This project was developed as part of a robotics automation system combining digital twin simulation, AI-based object recognition, and real-time ROS2 control.
