#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <SDL2/SDL.h>
#include <utility>

geometry_msgs::PoseStamped loc_pos;
void loc_pos_cb(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
	loc_pos = *msg;
}

//bool bat_aquired = false;
sensor_msgs::BatteryState bat;
void bat_cb(const sensor_msgs::BatteryState::ConstPtr & msg)
{
	//bat_aquired = true;
	bat = *msg;
}

geometry_msgs::PoseStamped target_pos;
void target_cb(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
	target_pos = *msg;
}

double GetYaw(const geometry_msgs::Quaternion & quat)
{
	double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
	double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
	return std::atan2(siny_cosp, cosy_cosp);
}

std::pair<int, int> GetPixelLoc(double x, double y)
{
	constexpr double pixCoef = 788.68;
	return std::make_pair(pixCoef * (-y - 0.64), pixCoef * (-x + 1.19));
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "visualization_node");
	ros::NodeHandle nh;

	std::string mavros_pose_node_name = "mavros/local_position/pose";
	std::string mavros_battery_node_name = "mavros/battery";
	std::string ppnode_name = "pp_node/target_pose";

	ros::Subscriber loc_sub = nh.subscribe<geometry_msgs::PoseStamped>
		(mavros_pose_node_name, 10, loc_pos_cb);
	ros::Subscriber bat_sub = nh.subscribe<sensor_msgs::BatteryState>
		(mavros_battery_node_name, 10, bat_cb);
	ros::Subscriber target_sub = nh.subscribe<geometry_msgs::PoseStamped>
		(ppnode_name, 10, target_cb);
//	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
//		("mavros/state", 10, state_cb);

	ros::Rate rate(20.0);
	
	ROS_INFO("Initializing SDL...");
	SDL_Window * window;
	SDL_Renderer * renderer;
	if(SDL_Init(SDL_INIT_VIDEO) != 0)
	{
		ROS_FATAL("Cannot initialize SDL library, %s", SDL_GetError());
		return -1;
	}
	window = SDL_CreateWindow("Visual", SDL_WINDOWPOS_CENTERED,
			SDL_WINDOWPOS_CENTERED,
			1920, 1200, SDL_WINDOW_FULLSCREEN_DESKTOP);
	if(window == nullptr)
	{
		ROS_FATAL("Cannot create window, %s", SDL_GetError());
		return -1;
	}
	renderer = SDL_CreateRenderer(window, -1, 0);
	if(renderer == nullptr)
	{
		ROS_FATAL("Cannot create renderer, %s", SDL_GetError());
		return -1;
	}
	ROS_INFO("Successfully initialized visualization system");
	//bool SDLQuit = false;
	
	SDL_Surface *s_label{nullptr}, *s_cursor{nullptr};
	SDL_Texture *t_label{nullptr}, *t_cursor{nullptr};
	std::string label_path = ros::package::getPath("gcserver");
	if(*label_path.rbegin() != '/')
		label_path += "/";

	s_label = SDL_LoadBMP((label_path + "drone_label.bmp").c_str());
	s_cursor = SDL_LoadBMP((label_path + "cursor.bmp").c_str());
	if(s_label == nullptr || s_cursor == nullptr)
	{
		ROS_FATAL("Cannot load bitmap %s, %s", label_path.c_str(), SDL_GetError());
		return -1;
	}
	t_label = SDL_CreateTextureFromSurface(renderer, s_label);
	t_cursor = SDL_CreateTextureFromSurface(renderer, s_cursor);
	if(t_label == nullptr || t_cursor == nullptr)
	{
		ROS_FATAL("Cannot create label texture, %s", SDL_GetError());
		return -1;
	}
	SDL_RendererFlip flip = SDL_FLIP_NONE;
	
	//while(ros::ok	

	while(ros::ok())
	{
		SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
		SDL_RenderClear(renderer);
		//SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
		
		// Draw the drone
		SDL_Rect drone;
		auto center = GetPixelLoc(loc_pos.pose.position.x, loc_pos.pose.position.y);
		drone.x = center.first - 128;
		drone.y = center.second - 128;
		drone.h = drone.w = 256;
		
		//if(drone.x < 0 || drone.y < 0)

		double yaw = -(GetYaw(loc_pos.pose.orientation) * 180 / M_PI + 90);
		int renderResult = 
			SDL_RenderCopyEx(renderer, t_label, nullptr,
					&drone, yaw, nullptr, flip);
		if(renderResult != 0)
		{
			ROS_ERROR("Cannot render the drone, %s", SDL_GetError());
		}
		
		// Draw the battery
		SDL_Rect battery;
		battery.x = center.first - 128, battery.y = center.second + 128;
		battery.w = 256, battery.h = 32;
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);			
		SDL_RenderFillRect(renderer, &battery);
		if(!(0 < bat.percentage && bat.percentage < 1))
		{
			//if(!bat_aquired)
			ROS_INFO("Ignoring battery level : %f", bat.percentage);
		}
		else
		{
			battery.w *= bat.percentage;
			SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
			SDL_RenderFillRect(renderer, &battery);
		}

		// Draw the target
		center = GetPixelLoc(target_pos.pose.position.x, target_pos.pose.position.y);
		drone.x = center.first - 64;
		drone.y = center.second - 64;
		drone.h = drone.w = 128;
		SDL_RenderCopyEx(renderer, t_cursor, nullptr, &drone, 0, 0, flip);

		SDL_RenderPresent(renderer);
		ros::spinOnce();
		rate.sleep();
	}
	
	SDL_DestroyTexture(t_label);
	SDL_DestroyTexture(t_cursor);
	SDL_FreeSurface(s_label);
	SDL_FreeSurface(s_cursor);

	SDL_DestroyWindow(window);
	SDL_DestroyRenderer(renderer);
	SDL_Quit();

	return 0;
}

