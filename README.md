# Turbobot-Autonomous-Robot

This repository showcases a four-wheel differential drive robot designed and built from scratch. It includes the 3D model design, ROS2 framework integration, localization using EKF, firmware implementation, and hardware bringup. Below are the key aspects of the project:

---

## Table of Contents

1. [3D Model Design](#3d-model-design)
2. [ROS2 Framework Configuration](#ros2-framework-configuration)
3. [ROS2 Localization with EKF](#ros2-localization-with-ekf)
4. [Turbobot Firmware Package](#turbobot-firmware-package)
5. [Turbobot Bringup Package](#turbobot-bringup-package)
6. [TODO](#todo)
7. [Media](#media)
8. [How to Run](#how-to-run)
9. [Contributing](#contributing)

---

## 3D Model Design

The robot chassis and components were designed in SolidWorks with a focus on robustness and simplicity. Key design features include:

- **Chassis**: Compact and durable, optimized for differential drive control.
- **Motors and Wheels**: Equipped with four 12V DC motors with encoders for precise movement and control.
- **Sensors**: Design accommodates IMU and encoders for accurate localization.

![SolidWorks Design](images/solidworks_design_diff_drive.png)

---


https://github.com/user-attachments/assets/6d96dd51-13d0-4719-9e16-4e17023ca4d6


## ROS2 Framework Configuration

The robot is integrated with the ROS2 framework to enable advanced robotic functionalities. The configuration is divided into:

### 1. Turbobot Description Package
- **URDF File**: Created a URDF file from the Solidworks to accurately describe its physical structure.
- **RViz Visualization**: Used RViz2 for real-time visualization of the robot model and its movements.

### 2. Turbobot Controller Package
- **Differential Drive Controller**: Configured the ROS2 control package to manage differential drive mechanics and handle wheel commands.
- **Joint State Publishing**: Ensures real-time feedback of wheel states for visualization and control.

---

## ROS2 Localization with EKF

Localization is implemented using the Extended Kalman Filter (EKF) through the `robot_localization` package. This setup fuses sensor data to provide robust state estimation:

- **Sensor Fusion**: Combines data from wheel encoders and an IMU sensor.
- **EKF Implementation**: Filters noisy sensor data for accurate pose and velocity estimation.

---

## Turbobot Firmware Package

The firmware developed for the robot provides seamless communication between hardware and software:

- **Serial Communication**: Reads commands from ROS2 control via serial interface.
- **Encoder Feedback**: Sends wheel encoder data back to ROS2 control for real-time updates.

[Video of Firmware Functionality](videos/turbobot_firmware_demo.mp4)

---

## Turbobot Bringup Package

The bringup package initializes and visualizes the robot’s hardware or simulation:

- **Hardware Setup**: Launch file to start the physical robot.
- **Simulated Setup**: RViz2 integration to visualize and test the robot in a simulated environment.

---

## TODO

Future improvements and features:

1. Add a LiDAR sensor for environment scanning.
2. Integrate `nav2` and `slam_toolbox` for autonomous navigation and SLAM.

---

## Media

### Images

- **SolidWorks Design**
  ![SolidWorks Design](images/solidworks_design_diff_drive.png)

### Videos

- [Firmware Functionality](videos/turbobot_firmware_demo.mp4)
- [Localization with EKF](videos/ekf_localization_demo.mp4)

---


## Contributing

Contributions are welcome! Feel free to submit issues or pull requests for improvements and additional features.

