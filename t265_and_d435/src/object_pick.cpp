/*
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

class Listener{
public:
    float x_dist;
    float y_dist;
    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};
void Listener::positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   x_dist = msg->pose.position.y;
   y_dist = msg->pose.position.x;
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

    ros::init(argc, argv, "listener");
    ROS_INFO("t265_tracking_obj_3");
    ros::NodeHandle n;
    ros::Rate loop_rate(20.0);
    Listener listener;
    ros::Subscriber sub = n.subscribe<geometry_msgs::PoseStamped>("target_position", 60, &Listener::positionCallback,&listener);


    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
    	ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.7;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = -0.7;
    pose.pose.orientation.w = -0.7;

    geometry_msgs::PoseStamped pose_down;
    pose_down.pose.position.x = 0;
    pose_down.pose.position.y = 0;
    pose_down.pose.position.z = 0.0;
    pose_down.pose.orientation.x = 0;
    pose_down.pose.orientation.y = 0;
    pose_down.pose.orientation.z = -0.7;
    pose_down.pose.orientation.w = -0.7;

    geometry_msgs::PoseStamped pose_target_up;
    pose_target_up.pose.position.x = 0;
    pose_target_up.pose.position.y = 0;
    pose_target_up.pose.position.z = 0.7;
    pose_target_up.pose.orientation.x = 0;
    pose_target_up.pose.orientation.y = 0;
    pose_target_up.pose.orientation.z = -0.7;
    pose_target_up.pose.orientation.w = -0.7;

    geometry_msgs::PoseStamped pose_target_down;
    pose_target_down.pose.position.x = 0;
    pose_target_down.pose.position.y = 0;
    pose_target_down.pose.position.z = 0.0;
    pose_target_down.pose.orientation.x = 0;
    pose_target_down.pose.orientation.y = 0;
    pose_target_down.pose.orientation.z = -0.7;
    pose_target_down.pose.orientation.w = -0.7;



    ROS_INFO("Sending initial setpoints");
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
	local_pos_pub.publish(pose);
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

    int count = 0;

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

	if(count < 500){
	    // initialize
	    ROS_INFO("Current count is %i, initializing hover",count);
        local_pos_pub.publish(pose_down);
	} else if (500 <= count && count < 700){
	    // go up	
	    ROS_INFO("Current count is %i, drone goes up",count);
        local_pos_pub.publish(pose);
	} else if (700 <= count && count < 800){
        // find object
	    ROS_INFO("Current count is %i, findind target",count);
	ROS_INFO("x = %.5f, y = %.5f",listener.x_dist,listener.y_dist);
        pose_target_up.pose.position.x = listener.x_dist;
        pose_target_up.pose.position.y = listener.y_dist;
        pose_target_down.pose.position.x = listener.x_dist;
        pose_target_down.pose.position.y = listener.y_dist;
        local_pos_pub.publish(pose);       
    } 	else if (800 <= count && count < 1000){
        // go above object
	    ROS_INFO("Current count is %i, going above target",count);
        local_pos_pub.publish(pose_target_up);       
    } 	else if (1000 <= count && count < 1300){
        // land on object
	    ROS_INFO("Current count is %i, landing on target",count);
        local_pos_pub.publish(pose_target_down);       
    } 	else if (1300 <= count && count < 1500){
        // go up
	    ROS_INFO("Current count is %i, going up",count);
        local_pos_pub.publish(pose_target_up);       
    } 	else if (1500 <= count && count < 1600){
        // return to origin up
	    ROS_INFO("Current count is %i, returning to origin",count);
        local_pos_pub.publish(pose);       
    } 	else if (1600 <= count && count < 1800){
        // land
	    ROS_INFO("Current count is %i, landing",count);
        local_pos_pub.publish(pose_down);       
    }
	else if (count >= 1800){
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

