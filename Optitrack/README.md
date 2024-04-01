## Drone control using Optitrack

Optitrack relies on a tracker fixed on top of the drone. Motive is used to create a rigid body (named here "body1") based on this tracker.

```bash
# Launch VRPN Client ROS Node
roslaunch vrpn_client_ros sample.launch server:="your_server"

# Launch MAVROS
roslaunch mavros px4.launch fcu_url:="your_fcu_url" gcs_url:=udp:"your_gcs_url"

# Relay VRPN Pose Topic to MAVROS
rosrun topic_tools relay /vrpn_client_node/body1/pose /mavros/vision_pose/pose

# Run Custom Script
rosrun Optitrack x_y_movement
