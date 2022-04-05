#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <csignal>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

/*
geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
    current_pose = *msg;
}
*/

geometry_msgs::PoseStamped target_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
	target_pose = *msg;
}

sig_atomic_t volatile g_request_shutdown = 0;
void onSigint(int signal)
{
	ROS_INFO("Requesting shutdown");
	g_request_shutdown = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    signal(SIGINT, onSigint);

	std::string pp_node_name = "pp_node/target_pose";

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    //ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //        ("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
			(pp_node_name, 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(!g_request_shutdown && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
	
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.y = 0;

    //send a few setpoints before starting
    for(int i = 100; !g_request_shutdown && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(!g_request_shutdown){
//	if(!current_state.manual_input){
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
//        }

        local_pos_pub.publish(target_pose);

        ros::spinOnce();
        rate.sleep();
    }

	offb_set_mode.request.custom_mode = "AUTO.LAND";
	set_mode_client.call(offb_set_mode);
	ROS_WARN("Engaging landing mode");
	if(!offb_set_mode.response.mode_sent)
		ROS_ERROR("Cannot enable landing mode");
	else
	{
		ROS_INFO("Waiting for 5 seconds");
		for(int i = 0; i < 100; i++)
			rate.sleep();
	}
	ros::shutdown();

    return 0;
}

