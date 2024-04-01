# Aerial_manipulation
This research project focuses on autonomous drone control for detecting and grabbing a target object based on its color with the assistance of a robotic hand mounted on the drone, utilizing depth data from a RealSense D435 camera and tracking capabilities of a T265 camera. The project is designed to run on a Jetson Nano using ROS Melodic and a px4 flight controller.

The control of the drone can be realized by two different ways:
- Using Optitrack, a motion planning system. This system doesn't include any object detection algorithm.
- Using the onboard tracking camera t265 (for control) and the depth camera d435 (for object detection).

In this resarch, **QGroundControl** has been used to calibrate the px4, **Motive** for Optitrack. The Jetson Nano is remotly controlled using **TeraTerm**.


## Dependencies

This project relies on the following external packages:

- [Intel RealSense ROS package](https://github.com/IntelRealSense/realsense-ros.git)
- [MAVROS package](https://github.com/mavlink/mavros.git)
- [vision_to_mavros package](https://github.com/thien94/vision_to_mavros.git)
- [vrpn_client_ros](https://github.com/ros-drivers/vrpn_client_ros.git)

Please make sure to install these packages before running the project.


## Installation

1. Clone this repository:
    ```bash
    git clone https://github.com/Etienne-Badard/Aerial_manipulation.git
    cd yourproject
    ```

2. Install dependencies:
    ```bash
    # Install Intel RealSense ROS package
    sudo apt-get install ros-melodic-realsense2-camera
    
    # Install MAVROS package
    sudo apt-get install ros-melodic-mavros
    ```

## Check the README file inside "Optitrack" or "t265_and_t435" folder for more specific informations
