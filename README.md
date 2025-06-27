# BIR-Intro-to-ROS2
Introduction to ROS2 Workshop materials and setup guide for the Black in Robotics Introduction to ROS2 Summer 2025 Workshop. 

# PC-SETUP

This repository provides a comprehensive guide to set up your system for the workshop. It includes instructions for system partitioning, software installations, ROS2 setup, and library configurations.

---

## **1. System Setup and Partitioning**

### Steps: (Dual-Boot) Highly recommended 
1. **Watch Tutorial:** Follow this [YouTube tutorial](https://youtu.be/Z-Hv9hOaKso?si=tka_nrAbiIuYnxvy) for partitioning and installing Ubuntu.
2. **Partitioning:** Configure your laptop for dual-booting with Ubuntu 22.04.
3. **Ubuntu Installation Guide:** Use the [Ubuntu 22.04 Installation Guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) for detailed steps.

###  Option 2: Use a Virtual Machine to Install Ubuntu 22.04
https://youtu.be/rJ9ysibH768


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
