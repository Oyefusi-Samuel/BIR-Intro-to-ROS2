![image](https://github.com/user-attachments/assets/0ea7c8a7-5757-4895-960a-acd816f881d6)


# BIR-Intro-to-ROS2
Introduction to ROS2 Workshop materials and setup guide for the Black in Robotics Introduction to ROS2 Summer 2025 Workshop. 

# PC-SETUP

This repository provides a comprehensive guide to set up your system for the workshop. It includes instructions for system partitioning, software installation, ROS 2 setup, and configuration of the Workshop Package.

---

## **1. System Setup and Partitioning**

### Steps: Option 1 (Dual-Boot) Highly recommended 
. **Watch Tutorial:** Follow this [YouTube tutorial](https://youtu.be/Z-Hv9hOaKso?si=tka_nrAbiIuYnxvy) for partitioning and installing Ubuntu.
. **Partitioning:** Configure your laptop for dual-booting with Ubuntu 22.04.
. **Ubuntu Installation Guide:** Use the [Ubuntu 22.04 Installation Guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) for detailed steps.

###  Option 2: Use a Virtual Machine to Install Ubuntu 22.04 
. Download and install VirtualBox from [here](https://www.virtualbox.org/wiki/Downloads).
. Go to [Ubuntu Downloads](https://ubuntu.com/download/desktop) and select Ubuntu 22.04 LTS.
. You can watch this [video](https://youtu.be/jm8WMgwu10s ) to understand the installation steps faster 
             

---

## **3. ROS2 Humble Installation**

### Steps:
1. **Follow Documentation:** Use the official [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
2. You can also use this Documentation for Clarity purposes https://robocre8.gitbook.io/robocre8/tutorials/how-to-install-ros2-humble-desktop-on-pc-full-install
3. **Post-Installation Check:** Verify the installation by running:
   ```bash
   ros2 --version
   ```

---




###  Option 3: Use DevContainer with VSCode + Docker (Beginner-friendly)- ROS2env 
1. Install [Visual Studio Code](https://code.visualstudio.com/)
2. Install [Docker Desktop](https://www.docker.com/products/docker-desktop/)
3. Follow this repo  [ROS2env](https://github.com/SakshayMahna/ros2env)
4. Open in VSCode → "Reopen in Container"  
   *(You'll now be inside a fully working ROS2 environment)*

> The `.devcontainer` folder is pre-configured with ROS2 Humble and common tools.



---

## **2. Software Setup**

### Steps:
1. **Install Ubuntu 22.04**
   - Ensure Ubuntu 22.04 is installed successfully on your system.

2. **Install Essential Developer Tools:**
   ```bash
   sudo apt update
   sudo apt install build-essential curl git
   ```

---

## **3. ROS2 Humble Installation**

### Steps:
1. **Follow Documentation:** Use the official [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
2. **Post-Installation Check:** Verify the installation by running:
   ```bash
   ros2 --version
   ```

---

## WORKSHOP PAKAGE [MOBO_BOT](https://github.com/robocre8/mobo_bot) BY [robocre8](https://github.com/robocre8)
Give the package a Star and learn alot about Ros and intro to robotics  https://github.com/robocre8/mobo_bot


## MoboBot Gazebo Simulation

![mobo_bot_slam](./docs/mobo_bot_slam_sim.gif)

> [!NOTE]
> Your Dev PC must be running **Ubuntu 22.04** and **ros-humble-desktop** with **gazebo igintion fortress**.
> </br>
> You can follow this [**tutorial**](https://robocre8.gitbook.io/robocre8/tutorials/how-to-install-ros2-humble-desktop-on-pc-full-install) to install **ros-humble-desktop** on **PC**
> </br>
> The **ignition gazebo** would be installed as you follow the installation process below.

#

### Some Prerequisites

- Install and set up Cyclone DDS on your PC (if you don't have it installed yet).
  ```shell
  sudo apt install ros-humble-rmw-cyclonedds-cpp
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
  ```
  
- Create your MoboBot ROS Workspace
  ```shell
  mkdir -p ~/mobo_bot_ws/src
  cd ~/mobo_bot_ws
  colcon build
  source ~/mobo_bot_ws/install/setup.bash
  ```

- Clone the **arrow_key_telop_drive** package on your MoboBot ROS Workspace. This is the package that would be used for driving the MoboBot using the arrow keys of your keyboard
  ```shell
  sudo apt install python3-pip
  pip3 install pynput
  pip3 install setuptools==58.2.0
  cd ~/mobo_bot_ws/src
  git clone https://github.com/samuko-things/arrow_key_teleop_drive.git
  ```
  Learn more about the [**arrow_key_teleop_drive**](https://github.com/samuko-things/arrow_key_teleop_drive)

- Build your workspace
  ```shell
  cd ~/mobo_bot_ws
  colcon build --symlink-install
  ```

### Clone and Build the MoboBot Packages  
- cd into the src folder of your mobo_bot_ws and download the **MoboBot** packages
  ```shell
  cd ~/mobo_bot_ws/src
  git clone -b humble https://github.com/robocre8/mobo_bot.git
  ```
  
- If you are not interested in running or testing the MoboBot hardware (i.e the actual robot), run the following command below. this will add the COLCON_IGNORE file to it.
  </br>If not, please go ahead and skip this, then check the [**Working with the Actual MoboBot**]() tutorial.
  ```shell
  cd ~/mobo_bot_ws/src/mobo_bot/mobo_bot_base
  touch COLCON_IGNORE
  ```

- cd into the root directory of your mobo_bot_ws and run rosdep to install all necessary ROS  package dependencies
  ```shell
  cd ~/mobo_bot_ws
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y
  ```

- Build your mobo_bot_ws
  ```shell
  cd ~/mobo_bot_ws
  colcon build --symlink-install
  ```

- Don't forget to source your <ros_ws> in any new terminal
  ```shell
  source ~/mobo_bot_ws/install/setup.bash
  ```

#

### View Robot and Transform Tree
![mobo_bot_tf](./docs/mobo_bot_tf.png)
- on your dev-PC, open a new terminal and start the robot state publisher node
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ros2 launch mobo_bot_description rsp.launch.py use_joint_state_pub:=true
  ```
- in a different terminal, run the rviz launch file to view the robot
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ros2 launch mobo_bot_rviz rsp.launch.py
  ```
- To view the transform tree, in a different terminal (while the robot state publisher is still running), run the following
  ```shell
  ros2 run rqt_tf_tree rqt_tf_tree
  ```

#

### Run the MoboBot simulation
- On your dev-PC, open a new terminal and start the mobo_bot_sim 
  ```shell
  source ~/mobo_bot_ws/install/setup.bash
  ros2 launch mobo_bot_bringup sim.launch.py
  ```
- In a different terminal, run the mobo_bot_teleop to drive the robot around using the arrow keys on your keyboard
  ```shell
  source ~/mobo_bot_ws/install/setup.bash
  ros2 run arrow_key_teleop_drive arrow_key_teleop_drive
  ```
  drive the robot easily using the arrow keys

```
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
  ```shell
  source ~/mobo_bot_ws/install/setup.bash
  ros2 run arrow_key_teleop_drive arrow_key_teleop_drive 0.3 0.9
  ```
  
