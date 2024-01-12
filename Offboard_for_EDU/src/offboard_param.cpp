#include <offboard_param.h>
#include <ros/ros.h>
#include <iostream>
#include <string>

using namespace std;
#define R2D 180/3.14159265359
#define D2R 3.14159265359/180.0

offboard_param::offboard_param(ros::NodeHandle& nh)
{
    std::string srv_name("/offboard_param");
    srv_name = "" + srv_name;
    this->service = nh.advertiseService(srv_name.c_str(), &offboard_param::callback_parmaUpdate, this);

    this->param[0] = 20;       // p gain - normal
    this->param[1] = 10;       // p gain - Y axis
    this->param[2] = 1.75;        // safety radius
}

bool offboard_param::callback_parmaUpdate(mavros_msgs::ParamSet::Request &req, mavros_msgs::ParamSet::Response &res)
{
    this->param[req.value.integer] = req.value.real;
	res.value.real = req.value.real;
	res.value.integer = req.value.integer;
	res.success = true;

	ROS_INFO(" [offboard] Parameter changing success ");
	return true;
}