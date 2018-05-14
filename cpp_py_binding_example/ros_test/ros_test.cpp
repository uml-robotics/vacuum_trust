#include <boost/python.hpp>
#include "ros/ros.h"
#include <iostream>
//include messages too
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

boost::python::object t_callback;

// Things to publish
sensor_msgs::LaserScan laser_scan_msg;
nav_msgs::Odometry odom_msg;



}
