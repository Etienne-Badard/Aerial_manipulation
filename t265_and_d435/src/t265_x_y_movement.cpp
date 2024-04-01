/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    ROS_INFO("Mode: %s, Armed: %d", current_state.mode.c_str(), static_cast<int>(current_state.armed));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    int count = 0;
    ROS_INFO("t265_x_y_movement");
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose_up;
    pose_up.pose.position.x = 0;
    pose_up.pose.position.y = 0;
    pose_up.pose.position.z = 0.7;
    pose_up.pose.orientation.x = 0;
    pose_up.pose.orientation.y = 0;
    pose_up.pose.orientation.z = -0.7;
    pose_up.pose.orientation.w = -0.7;

    geometry_msgs::PoseStamped pose_forward;
    pose_forward.pose.position.x = 0;
    pose_forward.pose.position.y = 0.40;
    pose_forward.pose.position.z = 0.7;
    pose_forward.pose.orientation.x = 0;
    pose_forward.pose.orientation.y = 0;
    pose_forward.pose.orientation.z = -0.7;
    pose_forward.pose.orientation.w = -0.7;

    geometry_msgs::PoseStamped pose_lateral;
    pose_lateral.pose.position.x = 0.40;
    pose_lateral.pose.position.y = 0;
    pose_lateral.pose.position.z = 0.7;
    pose_lateral.pose.orientation.x = 0;
    pose_lateral.pose.orientation.y = 0;
    pose_lateral.pose.orientation.z = -0.7;
    pose_lateral.pose.orientation.w = -0.7;

    geometry_msgs::PoseStamped pose_down;
    pose_down.pose.position.x = 0;
    pose_down.pose.position.y = 0;
    pose_down.pose.position.z = 0;
    pose_down.pose.orientation.x = 0;
    pose_down.pose.orientation.y = 0;
    pose_down.pose.orientation.z = -0.7;
    pose_down.pose.orientation.w = -0.7;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose_up);
        ROS_INFO("Sending set point for initialisation");
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){		
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if (count<500){
            local_pos_pub.publish(pose_down);
            ROS_INFO("Count = %i, drone in initial position !!!",count);
        }

	// Go up
        else if (500 <= count && count < 700){
            local_pos_pub.publish(pose_up);
            ROS_INFO("Count = %i, drone goes up",count);
        }

	// Go forward
        else if (count >= 700 && count < 900){
            local_pos_pub.publish(pose_forward);
            ROS_INFO("Count = %i, drone goes y = 0.4m",count);
        }

	// Come back in x,y = 0,0
        else if (count >= 900 && count < 1000){
            local_pos_pub.publish(pose_up);
            ROS_INFO("Count = %i, drone back to the center",count);
        }

	// Go side
        else if (count >= 1000 && count < 1200){
            local_pos_pub.publish(pose_lateral);
            ROS_INFO("Count = %i, drone goes x = 0.4",count);
        }

	// Come back in x,y = 0,0
        else if (count >= 1200 && count < 1300){
            local_pos_pub.publish(pose_up);
            ROS_INFO("Count = %i, drone back to the center",count);
        }

	// Go down
        else if (count >= 1300 && count < 1500){
            local_pos_pub.publish(pose_down);
            ROS_INFO("Count = %i, drone goes down",count);
        }


	else if (count >= 1500){
	   if( arming_client.call(disarm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle disarmed");}
        }
        ros::spinOnce();
        rate.sleep();
	count++;
    }

    return 0;
}

