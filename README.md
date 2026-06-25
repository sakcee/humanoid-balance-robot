# Humanoid Upper Body — Fall Detection & Anti-Disturbance Robot
### ROS 2 Humble | Gazebo | RViz | Ubuntu 22.04

A complete simulation of a humanoid upper-body robot featuring:
- **Real-time fall detection** using dual IMU sensors + Complementary Filter
- **Anti-disturbance PID controller** with feed-forward rejection
- **5-state machine**: STABLE → WARNING → FALLING → FALLEN → RECOVERY
- **Live RViz visualization** with color-coded state markers and torque arrows
- **Terminal dashboard** for real-time telemetry

---

## Prerequisites

```bash
sudo apt update && sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2
```

---

## Build

```bash
# Copy this package to your ROS 2 workspace
cp -r humanoid_balance_robot ~/ros2_ws/src/

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select humanoid_balance_robot
source install/setup.bash
```

---

## Run

### Full Simulation (Gazebo + RViz)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch humanoid_balance_robot humanoid_simulation.launch.py
```

### RViz Only (no Gazebo, for model preview)
```bash
ros2 launch humanoid_balance_robot humanoid_simulation.launch.py use_gazebo:=false
```

---

## Manual Testing

### Inject a disturbance (simulate a push)
```bash
ros2 topic pub /imu/torso sensor_msgs/msg/Imu \
  "{header: {frame_id: 'imu_torso_link'}, \
    angular_velocity: {x: 2.5, y: 1.0, z: 0.0}, \
    linear_acceleration: {x: 2.0, y: 9.0, z: 5.0}}" --once
```

### Monitor fall state
```bash
ros2 topic echo /fall_detection/state
```

### Monitor tilt angles
```bash
ros2 topic echo /fall_detection/angles
```

### Monitor controller torques
```bash
ros2 topic echo /joint_commands
```

### List all active topics
```bash
ros2 topic list
```

---

## Architecture

```
/imu/torso (200 Hz)
      │
      ▼
┌─────────────────────┐       ┌──────────────────────────┐
│  Fall Detection     │──────▶│  Anti-Disturbance        │
│  Node               │       │  Controller              │
│  - Comp. Filter     │       │  - PID (roll/pitch)      │
│  - State Machine    │       │  - Feed-forward          │
│  - Smoothing        │       │  - Emergency stop        │
└────────┬────────────┘       └──────────┬───────────────┘
         │                               │
         │   /fall_detection/state       │ /joint_commands
         │   /fall_detection/angles      │
         ▼                               ▼
    ┌──────────────────────────────────────┐
    │         Balance Monitor              │
    │   - Terminal dashboard               │
    │   - RViz HUD markers                 │
    └──────────────────────────────────────┘
```

## Fall Detection Algorithm

| Parameter        | Value     | Meaning                          |
|------------------|-----------|----------------------------------|
| Warning tilt     | 20°       | Start compensating               |
| Falling tilt     | 35°       | Aggressive correction            |
| Fallen tilt      | 60°       | Emergency stop                   |
| Angular vel.     | 1.5 rad/s | Rapid disturbance threshold      |
| Impact accel.    | 25 m/s²   | Impact/collision detection       |
| Filter alpha     | 0.98      | Complementary filter coefficient |

## Controller Parameters

| Parameter        | Roll    | Pitch   |
|------------------|---------|---------|
| Kp               | 25.0    | 22.0    |
| Ki               | 0.8     | 0.6     |
| Kd               | 4.5     | 4.0     |
| Max torque       | 45 Nm   | 45 Nm   |
| Feed-forward gain| 8.0     | 8.0     |

---

## Project by
**Sakshi** | Engineering Student | Robotics & Aerospace
Stack: ROS 2 Humble · C++17 · Gazebo · RViz · Ubuntu 22.04
