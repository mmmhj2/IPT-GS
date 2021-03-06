#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "apf.h"

bool ready = false;
geometry_msgs::PoseStamped currentPose;

inline double surface_eudistance(const geometry_msgs::Point & x1, const geometry_msgs::Point & x2)
{
	return sqrt((x1.x - x2.x) * (x1.x - x2.x) + (x1.y - x2.y) * (x1.y - x2.y));
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
	ready = true;
	currentPose = *msg;
}

//std::array <geometry_msgs::Point, 5> targets;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_planning_node");
	ros::NodeHandle nh, pnh("~");
	
	std::string target_pos_topic /*= "pp_node/target_pose"*/;
	std::string trajectory_topic /*= "pp_node/tarjectory"*/;
	std::string obstacles_topic;
	pnh.param<std::string>("TargetTopicName", target_pos_topic, "pp_node/target_pose");
	pnh.param<std::string>("TrajectoryTopic", trajectory_topic, "pp_node/trajectory");
	pnh.param<std::string>("ObstaclesTopic", obstacles_topic, "pp_node/obstacles");
	
	// Parameters
	double z_threshold;
	double z_epsilon;
	pnh.param<double>("z_threshold", z_threshold, 0.5);
	pnh.param<double>("z_epsilon", z_epsilon, 0.05);
	ROS_INFO("z_threshold %f, z_epsilon %f", z_threshold, z_epsilon);
	
	double coef_attr, coef_repl, dist_threshold;
	int max_iter;
	double march, tolerance;
	pnh.param<double>("coef_attraction", coef_attr, 1.0);
	pnh.param<double>("coef_replusion", coef_repl, 10.0);
	pnh.param<double>("distance_threshold", dist_threshold, 3);
	pnh.param<int>("max_iteration", max_iter, 5000);
	pnh.param<double>("march", march, 0.1);
	pnh.param<double>("tolerance", tolerance, 0.1);
	
	bool generate_traj;
	int traj_iteration, obstacle_period_multiplier;
	pnh.param<bool>("GenerateTrajectory", generate_traj, false);
	pnh.param<int>("TrajectoryIteration", traj_iteration, 10);
	pnh.param<int>("ObstaclePublishPeriodMultiplier", obstacle_period_multiplier, 20);

	APFHelper apfhelper(coef_attr, coef_repl, dist_threshold);
	ArtificialPotentialField apf(&apfhelper, max_iter, march, tolerance);
	
	
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
		("mavros/local_position/pose", 10, local_pos_cb);
	ros::Publisher target_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
		(target_pos_topic, 10);
	ros::Publisher traj_pub;
	if(generate_traj)
		traj_pub = nh.advertise<geometry_msgs::Polygon>(trajectory_topic, 10);
	ros::Publisher obstacle_pub = nh.advertise<geometry_msgs::Polygon>(obstacles_topic, 5);
	
		
	ros::Rate rate{20.0};
	
	geometry_msgs::PoseStamped targetPose;
	targetPose.pose.position.x = 0;
	targetPose.pose.position.y = 0;
	targetPose.pose.position.z = z_threshold;
	geometry_msgs::Polygon trajectory;
	
	// TODO : Use ROS service to set obstacles dynamically
	bool aquiredTarget = false;
	vector2d start{0,0}, end{1, 1};
	std::vector <vector2d> obs;
	pnh.param<double>("start_x", start.x, 0);
	pnh.param<double>("start_y", start.y, 0);
	pnh.param<double>("end_x", end.x, 1);
	pnh.param<double>("end_y", end.y, 1);
	// Processing obstacles
	XmlRpc::XmlRpcValue xrv;
	pnh.param("obstacles", xrv, xrv);
	for(int i = 0; i < xrv.size(); ++i)
	{
		vector2d ob;
		if(xrv[i].size() != 2)
			ROS_ERROR("Ill-formed parameter obstacles %d", i);
		else
			ob.x = xrv[i][0], ob.y = xrv[i][1], obs.push_back(ob);
	}
	
	ROS_INFO("Read %lu obstacles", obs.size());
	for(const auto & ob : obs)
		ROS_DEBUG("%f %f", ob.x, ob.y);

	while(ros::ok())
	{
		static int periodCounter = 0;
		ROS_DEBUG("Planning route");
		trajectory.points.clear();
		
		if(ready)
		{
			ROS_INFO_ONCE("Ready for planning");
			auto & position = currentPose.pose.position;
			// Aquire target location
			if(!aquiredTarget)
			{
				aquiredTarget = true;
				if(abs(end.x - position.x) + abs(end.y - position.y) <= 0.5)
				{
					std::swap(start, end);
					ROS_WARN("Swapped target location");
				}
			}
			
			if(position.z <= z_threshold - z_epsilon)
			{
				targetPose.pose.position = currentPose.pose.position;
				targetPose.pose.position.z = z_threshold;
				
				ROS_DEBUG("Retaining altitude : %f -> %f", position.z, z_threshold);
				if(generate_traj)
				{
					geometry_msgs::Point32 p;
					p.x = targetPose.pose.position.x;
					p.y = targetPose.pose.position.y;
					p.z = targetPose.pose.position.z;
					trajectory.points.push_back(std::move(p));
				}
			}
			else
			{
				vector2d position_;
				position_.x = position.x;
				position_.y = position.y;
				vector2d newtarget = apf.GetStep(position_, end, obs);
				targetPose.pose.position.x = newtarget.x;
				targetPose.pose.position.y = newtarget.y;
				targetPose.pose.position.z = z_threshold;
				ROS_INFO_THROTTLE(1, "Moving to : %f, %f", targetPose.pose.position.x, targetPose.pose.position.y);
				
				if(generate_traj)
				{
					geometry_msgs::Point32 p;
					p.x = targetPose.pose.position.x;
					p.y = targetPose.pose.position.y;
					p.z = targetPose.pose.position.z;
					trajectory.points.push_back(p);
					for(int i = 0; i < traj_iteration; ++i)
					{
						vector2d newPosition = apf.GetStep(newtarget, end, obs);
						p.x = newPosition.x, p.y = newPosition.y, p.z = z_threshold;
						trajectory.points.push_back(p);
						newtarget = newPosition;
					}
				}
					
			}
		}
		
		// Publish the obstacles
		if(periodCounter == 0)
		{
			geometry_msgs::Polygon poly;
			for(const auto & obstacle : obs)
			{
				geometry_msgs::Point32 p;
				p.x = obstacle.x;
				p.y = obstacle.y;
				p.z = 0;
				poly.points.push_back(std::move(p));
			}
			obstacle_pub.publish(poly);
		}
		
		target_pos_pub.publish(targetPose);
		if(generate_traj)
			traj_pub.publish(trajectory);
		
		periodCounter = (periodCounter + 1) % obstacle_period_multiplier;
		ros::spinOnce();
		rate.sleep();
	}
	
}

