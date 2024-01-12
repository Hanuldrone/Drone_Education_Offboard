#ifndef offboard_PARAM
#define offboard_PARAM

#include <vector>
#include <string>
#include <stdint.h>
#include <cmath>

#include <ros/ros.h>
#include <mavros_msgs/ParamSet.h>
#include <geometry_msgs/Point32.h>

class offboard_param
{
    private:
        bool callback_parmaUpdate(mavros_msgs::ParamSet::Request &req, mavros_msgs::ParamSet::Response &res);

    public:
        ros::ServiceServer service;
        double_t param[3];

    public:
        offboard_param(ros::NodeHandle& nh);
};

#endif

