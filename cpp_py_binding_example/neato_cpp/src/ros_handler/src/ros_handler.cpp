#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/python.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include "ros/ros.h"
#include <iostream>
#include <vector>

//include messages too
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h> //TODO broadcaster

boost::python::object t_callback;

// Things to publish
sensor_msgs::LaserScan laser_scan_msg;
nav_msgs::Odometry odom_msg;

bool destruct = false;
boost::thread spin_thread;

ros::Publisher l_pub; // ref to laser scan publisher
ros::Publisher o_pub;


void init_laser_msg();

void pub_laser(boost::python::list scans) //needs a list of tuples<double, double>
{
  laser_scan_msg.header.stamp = ros::Time::now(); //current time (is this what we want? or do we want time from the device)
  int i=0;
  int size = boost::python::len(scans);
  std::vector<float> f_ranges;
  std::vector<float> f_intensities;
  for (i; i<size; i++)
  {
    boost::python::tuple scan = boost::python::extract<boost::python::tuple>(scans.pop()); //casts element into tuple
    f_ranges.push_back((float)boost::python::extract<double>(scan[0])); //maybe don't need this conversion to float
    f_intensities.push_back((float)boost::python::extract<double>(scan[1]));
  }
  laser_scan_msg.ranges = f_ranges; //set laser scan
  laser_scan_msg.intensities = f_intensities;
  l_pub.publish(laser_scan_msg);
}


void pass_t_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // TODO do some handling of the msg to refactor, also test if non-primitives pass correctly (I doubt it), if not, get all components and send those
  if (t_callback)
  {
    t_callback(boost::python::make_tuple(msg->linear.x, msg->linear.y, msg->linear.z),
               boost::python::make_tuple(msg->angular.x, msg->angular.y, msg->angular.z));
  }
}



void dispose()
{
  destruct = true;
  spin_thread.join();
}

void ros_spin()
{
  while(!destruct)
  {
    boost::this_thread::sleep(boost::posix_time::millisec(200)); // TODO adjust
    ros::spinOnce();
  }
}


void initialize(boost::python::object twist_callback) //give it a python function
{
  init_laser_msg();
  t_callback = twist_callback;
  int argc = 2; //fake an argc argv
  char *argv[] = { "woof", NULL};
  ros::init(argc, argv, "neato");
  ros::NodeHandle n;
  std::cout << "Initialized";
  ros::Publisher l_pub = n.advertise<sensor_msgs::LaserScan>("base_scan", 10);  
//  ros::Publisher o_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  
  //subscribe to "cmd_vel", twist
  ros::Subscriber sub = n.subscribe("cmd_vel", 100, pass_t_callback);
  spin_thread = boost::thread(ros_spin);  
}



void init_laser_msg()
{
  //do header
  laser_scan_msg.header.seq = 1; //TODO
  laser_scan_msg.header.frame_id = "TODO"; //TODO
  laser_scan_msg.angle_min = 0.0f;
  laser_scan_msg.angle_max = 6.26f;
  laser_scan_msg.angle_increment = 0.017437326f;
  laser_scan_msg.range_min = 0.020f;
  laser_scan_msg.range_max = 5.0f;
}



BOOST_PYTHON_MODULE(ros_handler)
{
  using namespace boost::python;
  def("init", initialize);
  def("pub_laser", pub_laser);
  def("dispose", dispose);
}
