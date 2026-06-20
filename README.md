# ROS 2 Ackermann Navigation Ecosystem

A full autonomy stack for **Ackermann-steered rovers**, built from five specialized ROS 2 packages. The pipeline takes a 3D point-cloud map all the way through to physical motor commands — covering mapping, global relocalization, kinematically-feasible path planning, command translation, and hardware control.

> **Note:** This README is a high-level guide to how the packages fit together. Each package has its own repository with detailed build/install instructions — see the [Packages](#-packages) section for links.

---

## 🏗 System Architecture

The autonomous pipeline runs in five stages, each handled by a dedicated package:

```
 3D PointCloud2                  Global Pose                   Path                 Drive Cmd              Motor Cmd
 ───────────────►  [Mapping]  ───────────────►  [Localization]  ────►  [Path Planner]  ────►  [Control Bridge]  ────►  [Hardware Ctrl]
                   pcd_to_occupancy_grid        Reloc3D-ROS2_V1        ackermann_path_planner  ackermann_to_motor    ackermann_motor_ctrl
                        │                                                                                                    │
                        ▼                                                                                                    ▼
                    /map2d                                                                                          Arduino Mega 2560
                                                                                                                     (BTS7960 + Servo)
```

| Stage | Package | Role |
|---|---|---|
| 1 | `ROS2_pcd_to_occupancy_grid` | Converts 3D map → 2D occupancy grid |
| 2 | `Reloc3D-ROS2_V1` | Global relocalization on the 2D/3D map |
| 3 | `ros2_ackermann_path_planner` | Kinematically feasible Hybrid A* planning |
| 4 | `ros2_ackermann_to_motor` | Translates drive commands → hardware protocol |
| 5 | `ros2_ackermann_motor_ctrl` | Drives the physical motor/servo hardware |

A sixth package, `ros2_ackermann_keyboard_teleop`, provides manual control for testing (see [Manual Control & Testing](#-manual-control--testing)).

---

## 📦 Packages

### 1. Mapping & Processing — `ROS2_pcd_to_occupancy_grid`

Converts a 3D point cloud (e.g. from FAST-LIO or LIO-SAM) into a 2D map the planner can use.

- **Mapper Node** — subscribes to `/saved_map` (`sensor_msgs/PointCloud2`), slices it by height, and projects it into a 2D `nav_msgs/OccupancyGrid` published on `/map2d`.
- **Map GUI** — an interactive PyQt5 tool for **6-DOF alignment** (X, Y, Z, Roll, Pitch, Yaw), used to level and orient the map correctly before saving as `.pgm` and `.pcd` files.

### 2. Localization — `Reloc3D-ROS2_V1`

Gives the rover a reliable starting pose via global relocalization, rather than assuming it starts at the origin.

- **Algorithms** — **TEASER++** for robust global registration, **GICP** for continuous scan-to-map tracking.
- **TF Fusion** — manages the `map → camera_init` transform **[VERIFY: confirm this matches your SLAM backend's actual frame naming, e.g. `camera_init` vs `body` vs `lidar_init`]**, keeping it compatible with standard Nav2-style stacks.
- **Trigger** — a service call on `/relocalize_global` initiates global pose recovery.
- **Output** — publishes the recovered pose on `/relocalized_pose` (`geometry_msgs/PoseStamped`).

### 3. Path Planning — `ros2_ackermann_path_planner`

A **Hybrid A\*** planner built specifically for non-holonomic, Ackermann-steered robots.

- **Constraints** — respects the rover's **maximum turn radius**, so every generated path is physically driveable (no in-place turns or sharp pivots).
- **Input** — consumes `/map2d` and the relocalized pose from `/relocalized_pose`.
- **Output** — publishes the resulting waypoints as a `nav_msgs/Path` on `/path` **[VERIFY: confirm whether your planner publishes to `/path` or the Nav2-conventional `/plan` — pick whichever matches your actual node and keep it consistent with the topic table below]**.

### 4. Control Bridge — `ros2_ackermann_to_motor`

Bridges high-level navigation output with the low-level hardware protocol.

- **Input** — subscribes to `ackermann_msgs/msg/AckermannDriveStamped` on `/ackermann_cmd` **[VERIFY: confirm this is the exact topic name your bridge node subscribes to — update here and in the table if different]**, typically published by a path follower or the teleop package.
- **Translation** — converts each message into a string protocol understood by the Arduino firmware (e.g. `F150`, `B100`, `SERVO190`).
- **Output** — publishes these strings as `std_msgs/String` on `/motor_command`.

### 5. Hardware Control — `ros2_ackermann_motor_ctrl`

The final stage, interfacing directly with the physical rover.

- **Hardware** — a ROS 2 Python node talks to an **Arduino Mega 2560** over USB serial, driving a **BTS7960** motor driver and steering servo.
- **RC Override** — real-time switching between ROS-controlled autonomous mode and manual RC control via an FS-i6 transmitter (Channel 5).
- **Watchdog** — a 1-second software watchdog on the Arduino automatically stops the motor if the serial connection or ROS node fails.

---

## 🕹 Manual Control & Testing

For manual testing and initial setup, use `ros2_ackermann_keyboard_teleop` — a drop-in alternative to `teleop_twist_keyboard`, adapted for Ackermann steering constraints. It publishes `AckermannDriveStamped` messages on the same `/ackermann_cmd` topic **[VERIFY: matches Step 4's input topic]** that the control bridge consumes, so it can be used to exercise the bridge and hardware stack independently of the planner.

---

## 📋 Key Topics & Interfaces

| Component | Topic | Message Type | Purpose |
| :--- | :--- | :--- | :--- |
| Mapping | `/map2d` | `nav_msgs/OccupancyGrid` | 2D map for planning |
| Localization | `/relocalized_pose` | `geometry_msgs/PoseStamped` | Current robot pose in map |
| Planning | `/path` **[VERIFY]** | `nav_msgs/Path` | Feasible waypoints |
| Bridge (in) | `/ackermann_cmd` **[VERIFY]** | `ackermann_msgs/AckermannDriveStamped` | High-level drive command |
| Bridge (out) | `/motor_command` | `std_msgs/String` | Serial string for Arduino |

---

## 🚀 Quick Start

Bring the full stack up in order — each stage depends on the previous one being alive:

```bash
# 1. Build the 2D map from the saved 3D point cloud
ros2 launch ros2_pcd_to_occupancy_grid mapper.launch.py

# 2. Recover the rover's global pose on the map
ros2 launch reloc3d_ros2 relocalize.launch.py
ros2 service call /relocalize_global std_srvs/srv/Trigger

# 3. Start the path planner
ros2 launch ros2_ackermann_path_planner planner.launch.py

# 4. Start the control bridge (translates AckermannDriveStamped -> motor strings)
ros2 run ros2_ackermann_to_motor bridge_node

# 5. Start the hardware controller (talks to the Arduino over serial)
ros2 run ros2_ackermann_motor_ctrl motor_ctrl_node
```

**[VERIFY]** Launch file and executable names above are illustrative — replace with the actual filenames/entry points from each package.

For manual testing without the planner, swap step 3 for:

```bash
ros2 run ros2_ackermann_keyboard_teleop teleop_node
```

---

## 🛠 Prerequisites & Build

- **OS:** Ubuntu 22.04
- **ROS 2 distro:** Humble
- **Key libraries:** PCL, Eigen, TEASER++, OpenCV, PySerial
- **Arduino libraries:** `IBusBM`, `Servo` (install via Arduino IDE)

Each package has its own `README.md` and `install.md` with full build instructions and wiring diagrams — refer to those for package-specific setup.


---

## 🔌 Hardware Wiring

**[VERIFY / TODO]** Add a wiring diagram or link to one in the `ros2_ackermann_motor_ctrl` repo, covering:

- Arduino Mega 2560 ↔ BTS7960 motor driver connections
- Steering servo signal/power wiring
- FS-i6 receiver ↔ Arduino (Channel 5 override wiring)
- USB serial connection from the main compute board to the Arduino

---
