
https://github.com/user-attachments/assets/3c76466b-e391-4844-906c-7c05e12edf5a
# Turbobot-Autonomous-Robot

This repository showcases a four-wheel differential drive robot designed and built from scratch. It includes the 3D model design, ROS2 framework integration, localization using EKF, firmware implementation, and hardware bringup. Below are the key aspects of the project:


https://github.com/user-attachments/assets/a37dd53c-2390-4a10-aca8-deebb67d79c1


---

## Table of Contents

1. [3D Model Design](#3d-model-design)
2. [ROS2 Framework Configuration](#ros2-framework-configuration)
3. [ROS2 Localization with EKF](#ros2-localization-with-ekf)
4. [Turbobot Firmware Package](#turbobot-firmware-package)
5. [Turbobot Bringup Package](#turbobot-bringup-package)
6. [TODO](#todo)
7. [Contributing](#contributing)

---

## 3D Model Design

The robot chassis and components were designed in SolidWorks with a focus on robustness and simplicity. Key design features include:

- **Chassis**: Compact and durable, optimized for differential drive control.
- **Motors and Wheels**: Equipped with four 12V DC motors with encoders for precise movement and control.
- **Sensors**: Design accommodates IMU and encoders for accurate localization.


https://github.com/user-attachments/assets/6d96dd51-13d0-4719-9e16-4e17023ca4d6

---


## ROS2 Framework Configuration

The robot is integrated with the ROS2 framework to enable advanced robotic functionalities. The configuration is divided into:

### 1. Turbobot Description Package
- **URDF File**: Created a URDF file from the Solidworks to accurately describe its physical structure.
- **RViz Visualization**: Used RViz2 for real-time visualization of the robot model and its movements.

### 2. Turbobot Controller Package
- **Differential Drive Controller**: Configured the ROS2 control package to manage differential drive mechanics and handle wheel commands.
- **Joint State Publishing**: Ensures real-time feedback of wheel states for visualization and control.


https://github.com/user-attachments/assets/6af6fd65-e95c-474f-9c42-16ca92db2a46

https://github.com/user-attachments/assets/eaf015e2-bc32-4cbf-b950-43cc5cfae7bc

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



https://github.com/user-attachments/assets/950cfb9a-0e33-45bd-b91e-5be3077a2346



https://github.com/user-attachments/assets/f371da3b-ce15-4cfa-9a51-53758e793328


---

## Turbobot Bringup Package

The bringup package initializes and visualizes the robot’s hardware or simulation:

- **Hardware Setup**: Launch file to start the physical robot.
- **Simulated Setup**: RViz2 integration to visualize and test the robot in a simulated environment.


https://github.com/user-attachments/assets/525a8be4-f4b9-4b29-8b2c-a98935a81687


---

## TODO

Future improvements and features:

1. Add a LiDAR sensor for environment scanning.
2. Integrate `nav2` and `slam_toolbox` for autonomous navigation and SLAM.

---


## Contributing

Contributions are welcome! Feel free to submit issues or pull requests for improvements and additional features.

