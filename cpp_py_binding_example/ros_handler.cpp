#include <boost/python.hpp>
#include "ros/ros.h"
#include <iostream>
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

void pub_laser(boost::python::list<boost::tuple<double, double>> scans) //needs a list of tuples<double, double>
{
  laser_scan_msg.header.stamp = ros::time::now(); //current time (is this what we want? or do we want time from the device)
  int i=0;
  int size = scans.count(); //size of the list
  float[size] f_ranges;
  float[size] f_intensities;
  for (i; i<size; i++)
  {
    f_ranges[i] = (float)boost::get<0>(scans.pop());  //pop pops the list, then we get the 0th element in the tuple
    f_intensities[i] = (float)boost::get<1>(scans.pop()); // same as above
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
  dsetruct = true;
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


void init(boost::python::object twist_callback) //give it a python function
{
  init_laser_msg();
  t_callback = twist_callback;
  int argc = 1; //fake an argc argv
  char **argv = {"woof"};
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
  laser_scan_msg.header.frame_id = "TODO" //TODO
  laser_scan_msg.angle_min = 0.0f;
  laser_scan_msg.angle_max = 6.26f;
  laser_scan_msg.angle_increment = 0.017437326f;
  laser_scan_msg.range_min = 0.020f;
  laser_scan_msg.range_max = 5.0f;
}



BOOST_PYTHON_MODULE(ros_handler)
{
  using namespace boost::python;
  def("init", init);
  def("pub_laser", pub_laser);
}
