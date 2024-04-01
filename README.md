# Aerial_manipulation
This research project focuses on autonomous drone control for detecting and grabbing a target object with the assistance of a hand on the drone, utilizing depth data from a RealSense D435 camera and tracking capabilities of a T265 camera. The project is designed to run on a Jetson Nano using ROS Melodic.


## Dependencies

This project relies on the following external packages:

- [Intel RealSense ROS package](https://github.com/IntelRealSense/realsense-ros.git)
- [MAVROS package](https://github.com/mavlink/mavros.git)
- [vision_to_mavros package](https://github.com/thien94/vision_to_mavros.git)

Please make sure to install these packages before running the project.

## Installation

1. Clone this repository:
    ```bash
    git clone https://github.com/yourusername/yourproject.git
    cd yourproject
    ```

1. Install dependencies:
    ```bash
    # Install Intel RealSense ROS package
    sudo apt-get install ros-melodic-realsense2-camera
    
    # Install MAVROS package
    sudo apt-get install ros-melodic-mavros
    ```

## Usage

1. Launch the RealSense camera node:
    ```bash
    roslaunch realsense2_camera rs_t265.launch
    ```

2. Launch MAVROS:
    ```bash
    roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600 gcs_url:=udp://@192.168.100.2:14550
    ```

3. Launch vision_to_mavros:
    ```bash
    roslaunch vision_to_mavros t265_tf_to_mavros.launch
    ```

4. Run the custom scripts:
    ```bash
    rosrun scripts streming_depth_no_image.py
    rosrun scripts object_pick
    ```
