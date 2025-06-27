![image](https://github.com/user-attachments/assets/0ea7c8a7-5757-4895-960a-acd816f881d6)


# BIR-Intro-to-ROS2

Introduction to ROS2 Workshop materials and setup guide for the Black in Robotics Introduction to ROS2 Summer 2025 Workshop.

---

## 📁 PC SETUP

This section provides a comprehensive guide to set up your system for the workshop, including partitioning, ROS2 setup, and configuration of the development environment.

---

### 1. System Setup and Configuration 

#### 🔸 Option 1: Dual Boot (Highly Recommended)

- **Watch Tutorial:** [YouTube Tutorial](https://youtu.be/Z-Hv9hOaKso?si=tka_nrAbiIuYnxvy)
- **Partitioning:** Configure for dual-booting with Ubuntu 22.04.
- **Ubuntu Installation Guide:** [Ubuntu 22.04 Installation Guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)

#### 🔸 Option 2: Virtual Machine Installation

- **VirtualBox:** [Download VirtualBox](https://www.virtualbox.org/wiki/Downloads)
- **Ubuntu ISO:** [Ubuntu 22.04 LTS](https://ubuntu.com/download/desktop)
- **VM Tutorial:** [Video Guide](https://youtu.be/jm8WMgwu10s)

#### 🔸 Option 3: DevContainer with VSCode + Docker (Beginner Friendly)

- **Visual Studio Code:** [Install VSCode](https://code.visualstudio.com/)
- **Docker Desktop:** [Install Docker](https://www.docker.com/products/docker-desktop/)
- **Follow this repo:** [ROS2env](https://github.com/SakshayMahna/ros2env)
- In VSCode: Click `Reopen in Container`

> The `.devcontainer` folder is pre-configured with ROS2 Humble and tools.

---

### 2. Software Setup

- **Essential Developer Tools:**
  ```bash
  sudo apt update
  sudo apt install build-essential curl git
  ```

---

### 3. ROS2 Humble Installation

- **Primary Guide:** [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- **Alternative Documentation:** [Robocre8 Install Guide](https://robocre8.gitbook.io/robocre8/tutorials/how-to-install-ros2-humble-desktop-on-pc-full-install)
- **Post-Installation Check:**

  ```bash
  ros2 --version
  ```

---

![Uploading image.png…]()


## 🧰 Workshop Package: [MOBO_BOT](https://github.com/robocre8/mobo_bot)

Give the package a ⭐ and learn more about ROS and robotics.

---

### MoboBot Gazebo Simulation

![mobo_bot_slam](./docs/mobo_bot_slam_sim.gif)

> 📝 **Note:** Dev PC must be running **Ubuntu 22.04**, **ros-humble-desktop**, and **Gazebo Ignition Fortress**.

---

### 🔧 Prerequisites

- **Cyclone DDS Setup:**
  ```bash
  sudo apt install ros-humble-rmw-cyclonedds-cpp
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
  ```

- **Create Workspace:**
  ```bash
  mkdir -p ~/mobo_bot_ws/src
  cd ~/mobo_bot_ws
  colcon build
  source ~/mobo_bot_ws/install/setup.bash
  ```

- **Clone Arrow Key Teleop Package:**
  ```bash
  sudo apt install python3-pip
  pip3 install pynput
  pip3 install setuptools==58.2.0
  cd ~/mobo_bot_ws/src
  git clone https://github.com/samuko-things/arrow_key_teleop_drive.git
  ```

- **Build Workspace:**
  ```bash
  cd ~/mobo_bot_ws
  colcon build --symlink-install
  ```

---

### 📦 Clone and Build MoboBot

- **Clone MoboBot Packages:**
  ```bash
  cd ~/mobo_bot_ws/src
  git clone -b humble https://github.com/robocre8/mobo_bot.git
  ```

- **Ignore Hardware (optional):**
  ```bash
  cd ~/mobo_bot_ws/src/mobo_bot/mobo_bot_base
  touch COLCON_IGNORE
  ```

- **Install Dependencies:**
  ```bash
  cd ~/mobo_bot_ws
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y
  ```

- **Final Build:**
  ```bash
  colcon build --symlink-install
  ```

- **Source in New Terminals:**
  ```bash
  source ~/mobo_bot_ws/install/setup.bash
  ```

---

### 🔭 View Robot and TF Tree

- **Robot State Publisher:**
  ```bash
  ros2 launch mobo_bot_description rsp.launch.py use_joint_state_pub:=true
  ```

- **Rviz Visualization:**
  ```bash
  ros2 launch mobo_bot_rviz rsp.launch.py
  ```

- **TF Tree:**
  ```bash
  ros2 run rqt_tf_tree rqt_tf_tree
  ```

---

### 🎮 Run MoboBot Simulation

- **Start Simulation:**
  ```bash
  ros2 launch mobo_bot_bringup sim.launch.py
  ```

- **Control with Keyboard:**
  ```bash
  ros2 run arrow_key_teleop_drive arrow_key_teleop_drive
  ```

```text
---------------------------------------------------
drive around with arrow keys:

  [forward-left]  [forward]    [forward-right]
                      |
  [rotate left] -------------- [rotate right]
                      |
  [reverse-left]  [reverse]    [reverse-right]

stops when no arrow key is pressed

R - reset to default speed

Q - increase v by +0.05
Z - reduce v by -0.05

W - increase w by +0.1
X - reduce w by -0.1
----------------------------------------------------
```

- **Optional Speed Parameters:**
  ```bash
  ros2 run arrow_key_teleop_drive arrow_key_teleop_drive 0.3 0.9
  ```

---
