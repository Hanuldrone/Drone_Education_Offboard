#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <offboard_param.h>

#define RESET "\033[0m"
#define RED "\033[31m" /* Red */

#define LOOP_HZ 20.0f
#define D2R 3.14159265359 / 180.0
using namespace std;

mavros_msgs::State current_uavState;
void callback_uavState(const mavros_msgs::State::ConstPtr& msg)
{
  current_uavState = *msg;
}

std_msgs::Float64 uav_heading;
void callback_heading(const std_msgs::Float64::ConstPtr& msg)
{
  uav_heading = *msg;
}

geometry_msgs::Point32 cmd_vel_obs;
geometry_msgs::Point32 cmd_vel_obs_ned;
sensor_msgs::LaserScan lidar_data;

mavros_msgs::SetMode set_mode;

bool avoid_cal;
float rs = 1.75;        // safety circle radius
float k = 0.5;         // gain of u
float ky = 10;          // gain of velocity y axis
    
void callback_lidar(const sensor_msgs::LaserScan::ConstPtr& msg)
{ 
  lidar_data = *msg;      // lidar data from ros topic 

  int ic = 0;    
  bool avoid_cal = false; 

  for (ic = 0; ic < lidar_data.ranges.size(); ic++)
  {
    if (lidar_data.ranges[ic] < rs && lidar_data.ranges[ic] > 0.5)
    {
      avoid_cal = true;
      cmd_vel_obs.x = 0.25;

      float lx = cos(lidar_data.angle_increment * ic);  // x of lidar raw data
      float ly = sin(lidar_data.angle_increment * ic);  // y of lidar raw data
      float Ep = - k * 0.5 * pow(( 1/lidar_data.ranges[ic] - 1/rs), 2);         // potential energy by Obstacles
      cmd_vel_obs.x = cmd_vel_obs.x + Ep * lx;     // command velocity body X
      cmd_vel_obs.y = cmd_vel_obs.y + Ep * ly;     // command velocity body y

      if (cmd_vel_obs.x < 0)                          
      {
        cmd_vel_obs.x = 0;
        cmd_vel_obs.y = cmd_vel_obs.y + Ep * ly * ky;
      }
      // velocity saturation of y axis
      if (cmd_vel_obs.y < -2)                         
      {
        cmd_vel_obs.y = -2;
      }
      else if (cmd_vel_obs.y > 2)
      {
        cmd_vel_obs.y = 2;
      }
      // big gain values of velocity y when obstacles infront of drone. so it can avoid obstacle.
      if (lidar_data.ranges[lidar_data.ranges.size() * 0.5] < 1.5 ||        
          lidar_data.ranges[lidar_data.ranges.size() * 0.45] < 1.5 ||
          lidar_data.ranges[lidar_data.ranges.size() * 0.55] < 1.5)
      {
        cmd_vel_obs.x = -0.2;
        cmd_vel_obs.y = cmd_vel_obs.y + Ep * ly * ky;
      }

      // velocity saturation of y axis when drone between obstacles. and, reduce ocilation little bit.
      if (lidar_data.ranges[lidar_data.ranges.size() * 0.25] < rs)
      {
        if (cmd_vel_obs.y < -1)
        {
          cmd_vel_obs.y = -1;
        }
      }
      else if (lidar_data.ranges[lidar_data.ranges.size() * 0.75] < rs)
      {
        if (cmd_vel_obs.y > 1)
        {
          cmd_vel_obs.y = 1;
        }
      }  
    }
  }

  if (avoid_cal)
  {
    // rotate XYZ to NED
    cmd_vel_obs_ned.x = cmd_vel_obs.x * cos(uav_heading.data * D2R) - cmd_vel_obs.y * sin(uav_heading.data * D2R);
    cmd_vel_obs_ned.y = cmd_vel_obs.x * sin(uav_heading.data * D2R) + cmd_vel_obs.y * cos(uav_heading.data * D2R);

    // set mode offboard
    set_mode.request.custom_mode = "OFFBOARD";
  }

  else
  { 
    // Reset command velocity values
    cmd_vel_obs.x = cmd_vel_obs.x * 0.3;
    cmd_vel_obs.y = cmd_vel_obs.y * 0.25;
    cmd_vel_obs_ned.x = 0;
    cmd_vel_obs_ned.y = 0;

    // set mode mission
    set_mode.request.custom_mode = "AUTO.MISSION";
  }
}

void ros_start(int argc, char* argv[])
{
  ros::init(argc, argv, "offboard_node");
  ros::NodeHandle nh;
  ros::Rate rate(LOOP_HZ);

  // Publish & Serivice Client
  ros::Publisher cmdVel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1);
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // Subscribe
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, callback_uavState);
  ros::Subscriber hdg_sub = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10, callback_heading);
  ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, callback_lidar);



  // Define variables
  geometry_msgs::Point32 temp_cmd;
  geometry_msgs::Point32 temp_cmd_ned;
  geometry_msgs::Point32 cmd_vel_ned;
  geometry_msgs::TwistStamped setVel_msg;

  // wait for FCC(PX4) connection
  while (ros::ok() && !current_uavState.connected)
  {
    ros::spinOnce();
    rate.sleep();
    std::cout << "FCC connection success" << std::endl;
  }

  try
  {
    offboard_param gd_param(nh);
    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
      for (int i = 0; (i < LOOP_HZ) && ros::ok(); i++)
      {
        set_mode_client.call(set_mode) && set_mode.response.mode_sent;

        temp_cmd.x = 0.2;  // body X
        temp_cmd.y = 0;    // body Y

        k = gd_param.param[0];        
        ky = gd_param.param[1]; 
        rs = gd_param.param[2];    

        // rotate XYZ to NED
        temp_cmd_ned.x = temp_cmd.x * cos(uav_heading.data * D2R) - temp_cmd.y * sin(uav_heading.data * D2R);
        temp_cmd_ned.y = temp_cmd.x * sin(uav_heading.data * D2R) + temp_cmd.y * cos(uav_heading.data * D2R);
      }

      setVel_msg.twist.linear.x = temp_cmd_ned.y + cmd_vel_obs_ned.y;  // East
      setVel_msg.twist.linear.y = temp_cmd_ned.x + cmd_vel_obs_ned.x;  // North      
      cmdVel_pub.publish(setVel_msg);

      std::cout << "---------------- Current State and Command ---------------" << std::endl;
      std::cout << "uav heading(deg)                     :   " << uav_heading.data << std::endl;
      std::cout << "current mode                         :   " << current_uavState.mode << std::endl;
      std::cout << "velocity command - North             :   " << setVel_msg.twist.linear.y << std::endl;
      std::cout << "velocity command - East              :   " << setVel_msg.twist.linear.x << std::endl;
      std::cout << "lidar range [forward left]           :   " << lidar_data.ranges[lidar_data.ranges.size() * 0.55] << std::endl;
      std::cout << "lidar range [forward]                :   " << lidar_data.ranges[lidar_data.ranges.size() * 0.5] << std::endl;
      std::cout << "lidar range [forward right]          :   " << lidar_data.ranges[lidar_data.ranges.size() * 0.45] << std::endl;
      std::cout << "lidar range [right]                  :   " << lidar_data.ranges[lidar_data.ranges.size() * 0.25]<< std::endl;
      std::cout << "lidar range [left]                   :   " << lidar_data.ranges[lidar_data.ranges.size() * 0.75]<< std::endl;
      std::cout << "----------------------------------------------------------" << std::endl;
      rate.sleep();
      ros::spinOnce();
    }
  }
  catch (std::domain_error& e)
  {
    std::cout << RED << "[ERROR] " << e.what() << RESET << std::endl;
  }
}

int main(int argc, char* argv[])
{
  try
  {
    ros_start(argc, argv);
  }
  catch (std::domain_error& e)
  {
    std::cout << RED << "[ERROR] " << e.what() << RESET << std::endl;
  }

  return 0;
}
