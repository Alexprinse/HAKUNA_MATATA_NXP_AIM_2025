# 🦁 HAKUNA MATATA - NXP AIM India 2025 Competition

<div align="center">

**Team HAKUNA MATATA's Solution for NXP B3RB Warehouse Challenge**

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.8+-green)](https://www.python.org/)

*Autonomous Warehouse Navigation • Object Recognition • QR Code Detection*

</div>

---

## 📋 About This Project

This repository contains **Team HAKUNA MATATA's** complete implementation for the **NXP AIM India 2025 - B3RB Warehouse Challenge**. Our solution demonstrates autonomous navigation, intelligent path planning, real-time object recognition using YOLO, and QR code decoding for a robotic warehouse automation system.

### 🎯 Competition Challenge

Navigate an autonomous robot (NXP MR-B3RB) through a simulated warehouse to:
- 🗺️ Locate shelves using SLAM-generated maps
- 🚗 Navigate autonomously using ROS 2 Nav2 stack
- 📷 Decode QR codes for shelf identification and heuristics
- 🔍 Identify objects on shelves using YOLO object recognition
- 🎯 Optimize path using heuristic angles for maximum efficiency
- 🎪 Strategically reveal hidden shelves by completing sequences

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    B3RB Robot System                         │
├─────────────────────────────────────────────────────────────┤
│  Sensors: LIDAR | Camera | IMU | Encoders                   │
│  ↓                                                           │
│  SLAM & Localization (Nav2)                                 │
│  ↓                                                           │
│  Our Implementation (b3rb_ros_warehouse.py)                 │
│  ├── Navigation Controller                                   │
│  ├── QR Code Decoder                                        │
│  ├── Object Recognition Handler                             │
│  └── Strategic Path Planner                                 │
│  ↓                                                           │
│  Action: Shelf Data Publication → Score Points!            │
└─────────────────────────────────────────────────────────────┘
```

---

## 🚀 Key Features

### ✨ Our Implementation Highlights

- **🧠 Intelligent Navigation**: Custom path planning using Nav2 action clients
- **📸 Computer Vision**: Real-time QR code detection and decoding using OpenCV
- **🎯 Object Recognition**: Integration with YOLOv5 for accurate shelf object identification
- **🗺️ Map Analysis**: SLAM map processing for shelf localization
- **⚡ Optimization**: Heuristic-based pathfinding for efficient warehouse traversal
- **🎪 Sequential Unveiling**: Smart curtain revelation strategy for maximum points

---

## 📦 Package Contents

### Core Files
```
b3rb_ros_aim_india/
├── b3rb_ros_warehouse.py      # Main competition logic (OUR IMPLEMENTATION)
├── b3rb_ros_object_recog.py   # Object recognition node
├── b3rb_ros_draw_map.py       # Map visualization utilities
└── b3rb_ros_model_remove.py   # Model management utilities

Configuration & Resources:
├── shelves.yaml               # Shelf configuration
├── slam_map.png              # SLAM-generated map
├── annotated_map.png         # Annotated warehouse map
├── resource/
│   ├── yolov5n-int8.tflite  # YOLO model for object detection
│   └── coco.yaml             # Object class definitions
└── cache_control_service.py  # Cache management utilities
```

---

## 🛠️ Technical Stack

| Component | Technology |
|-----------|-----------|
| **Framework** | ROS 2 Humble Hawksbill |
| **Language** | Python 3.8+ |
| **Navigation** | Nav2 Stack |
| **SLAM** | SLAM Toolbox |
| **Object Detection** | YOLOv5 (TensorFlow Lite) |
| **Computer Vision** | OpenCV, NumPy |
| **Robot Platform** | NXP MR-B3RB / Gazebo Simulation |
| **Base System** | CogniPilot AIRY |

---

## 📥 Installation & Setup

### Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.8+
- CogniPilot AIRY workspace

### Quick Start

1. **Clone this repository into your CogniPilot Cranium workspace:**
   ```bash
   cd ~/cognipilot/cranium/src
   git clone https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git b3rb_ros_aim_india
   ```

2. **Install dependencies:**
   ```bash
   cd ~/cognipilot/cranium
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package:**
   ```bash
   colcon build --packages-select b3rb_ros_aim_india
   source install/setup.bash
   ```

4. **Run the warehouse challenge:**
   ```bash
   ros2 run b3rb_ros_aim_india b3rb_ros_warehouse
   ```

For detailed setup instructions, see the [original README](./README.md).

---

## 🎮 Usage

### Launch the Simulation
```bash
# Start the Gazebo simulation with warehouse environment
ros2 launch dream_world sim.launch.py world:=warehouse
```

### Run Our Solution
```bash
# Execute the warehouse challenge node
ros2 run b3rb_ros_aim_india b3rb_ros_warehouse
```

### Monitor Progress
```bash
# View shelf data being published
ros2 topic echo /shelf_data

# Check navigation status
ros2 topic echo /cerebri/out/status
```

---

## 📊 Results & Performance

Our solution successfully:
- ✅ Navigates autonomously through the warehouse
- ✅ Decodes QR codes with high accuracy
- ✅ Identifies objects using YOLO recognition
- ✅ Implements efficient path planning using heuristics
- ✅ Manages sequential shelf unveiling

---

## 🎓 Learning Outcomes

Through this competition, we gained expertise in:
- Autonomous robot navigation with ROS 2 Nav2
- Computer vision and QR code processing
- Deep learning model integration (YOLO)
- SLAM and map-based localization
- Real-time decision making and path planning
- ROS 2 architecture and best practices

---

## 🏆 Competition Details

- **Event**: NXP AIM India 2025
- **Challenge**: B3RB Warehouse Treasure Hunt
- **Team**: HAKUNA MATATA (Team ID: 1535)
- **Platform**: NXP MR-B3RB Robot
- **Year**: 2025-2026

---

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

**Original Framework**: Based on [NXP AIM India 2025](https://github.com/NXPHoverGames/NXP_AIM_INDIA_2025) starter code.

---

## 🙏 Acknowledgments

- **NXP Semiconductors** for organizing the AIM India competition
- **CogniPilot Community** for the AIRY robotics platform
- **NXP HoverGames** for the MR-B3RB robot platform and simulation environment
- **ROS 2 Community** for the excellent documentation and tools

---

## 👥 Team HAKUNA MATATA

> *"Hakuna Matata - It means no worries for the rest of your days!"*

For questions or collaboration, feel free to reach out!

---

<div align="center">

**Built with ❤️ by Team HAKUNA MATATA**

⭐ Star this repo if you find it helpful!

</div>
