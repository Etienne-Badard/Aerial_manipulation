## This folder contain the different files related to the usage of the depth (d435) and tracking (t265) cameras.

## Usage for aerial manipulation mission: 

1. Launch the RealSense camera node:
    ```bash
    roslaunch realsense2_camera rs_t265.launch
    ```

2. Launch MAVROS:
    ```bash
    roslaunch mavros px4.launch fcu_url:="your_fcu_URL" gcs_url:="your_gcs_URL"
    ```

3. Launch vision_to_mavros:
    ```bash
    roslaunch vision_to_mavros t265_tf_to_mavros.launch
    ```

4. Object detection based on color and estiamation of the targeted object position:
    ```bash
    rosrun t265_and_d435 streaming_depth.py
    ```
5. Autonomous drone control depending on targeted object position 
    ```bash
    rosrun t265_and_d435 object_pick
    ```
    
## ColorPicker file

This file is designed to determined the precise HSV value of a targeted object. These values can then be used in the custom scripts to specify the color of the object we want to grasp.
