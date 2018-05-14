#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/python.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <cmath>

//include messages too
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>


#define PI 3.14159265
#define BASE_WIDTH 248 //mm

typedef boost::python::list pylist;
#define pyextract boost::python::extract

boost::python::object t_callback; //callback for receive cmd_vel

// Things to publish
sensor_msgs::LaserScan laser_scan_msg;
nav_msgs::Odometry odom_msg;

bool destruct;
boost::thread spin_thread;

ros::NodeHandle *h;
ros::Publisher l_pub; // ref to laser scan publisher
ros::Publisher o_pub;
ros::Subscriber sub;

int old_left, old_right; //starts 0
ros::Time old_time; //starts w/ current time
double x, y, theta; // all starts 0

int dbg = 0; //debug log level, for now it's just used as a bool

/*
   Data structures to be used (note, we can't enforce in code b/c python
   is not strongly typed)

Laser: list<list<double> > where inner list has size = 2
example creation:
------------- C++
boost::python::list outer_list;
boost::python::list inner_list_1;
inner_list_1.append(double_range_1);
inner_list_1.append(double_intensity_1);
//inner_list_1 has size = 2

outer_list.append(inner_list_1);    
// ... and so on to add more range/intensity pairs
------------- python
laser = [ [range_1, intensity_1], [range_2, intensity_2], ...]
#and so on
-------------

Odometry: int left, int right

 */


void init_laser_msg();
void init_odom_msg();
void init_transform();

void pub_laser(pylist scans) //needs a list of list
{
  int i=0;
  int size = boost::python::len(scans);
  if (dbg)
  {
    std::cout << "pub_laser: called with a list of size: " << size << "\n";
  }
  laser_scan_msg.header.stamp = ros::Time::now(); //current time (is this what we want? or do we want time from the device)

  std::vector<float> f_ranges;
  std::vector<float> f_intensities;
  int checksum = 0;
  for (i; i<size; i++)
  {
    pylist scan = pyextract<pylist>(scans[0]);
    f_intensities.push_back(pyextract<double>(scan[1]));
    f_ranges.push_back(pyextract<double>(scan[0]));
    checksum += f_ranges.back() + f_intensities.back();
  }
  laser_scan_msg.ranges = f_ranges; //set laser scan
  laser_scan_msg.intensities = f_intensities;
  if (dbg)
  {
    std::cout << "pub_laser: Publishing with checksum = " << checksum << "\n";
  }
  l_pub.publish(laser_scan_msg);
}


void pub_transform(int left, int right) //TODO discuss data structure
{
  if (dbg)
  {
    std::cout << "pub_transform: called with args: " << left << ", " << right << "\n";
  }
  static tf::TransformBroadcaster br; //can maybe move to global for less overhead
  tf::Transform transform;
  int d_left = left-old_left;
  int d_right = right-old_right;
  old_left = left; //update old values to the current one to prepare for next use
  old_right = right;

  double d_x = (d_left+d_right)/2000; // /2 for avg, /1000 to convert uints
  double d_theta = (d_right-d_left)/BASE_WIDTH;
  double xx = cos(d_theta)*d_x;
  double yy = -sin(d_theta)*d_x;
  x += cos(theta)*xx - sin(theta)*yy;
  y += sin(theta)*xx + cos(theta)*yy;
  theta += d_theta;

  transform.setOrigin(tf::Vector3(x,y,0));
  tf::Quaternion q(0,0,sin(theta/2),cos(theta/2));
  transform.setRotation(q);
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.orientation.z = sin(theta/2);
  odom_msg.pose.pose.orientation.w = cos(theta/2);

  ros::Time time = ros::Time::now();
  ros::Duration d_tros = time-old_time;
  double d_t = d_tros.toSec();
  odom_msg.twist.twist.linear.x = d_x/d_t;
  odom_msg.twist.twist.angular.z = d_theta/d_t;  old_time = time;
  odom_msg.header.stamp = time;
  if (dbg)
  {
    std::cout << "pub_transform: Publishing with x=" << x << ", y=" << y << "q=(0,0," << sin(theta/2) << "," << cos(theta/2) << ")\n";
  }
  o_pub.publish(odom_msg);
  br.sendTransform(tf::StampedTransform(transform, time, "base_link", "odom"));
}

void pass_t_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  std::cout << "callback fired\n" << std::flush;
  if (t_callback)
  {
    //      int a = 1;
    //      int b = 2;

    //boost::python::PyGILState_STATE gstate;
    PyGILState_STATE gstate;
    std::cout << "init gstate\n" << std::flush;
    gstate = PyGILState_Ensure();
    std::cout << "gil ensure\n" << std::flush;
    //        boost::python::object a = boost::python::make_tuple((double)msg->linear.x, (double)msg->linear.y, (double)msg->linear.z);
    //        boost::python::object b = boost::python::make_tuple((double)msg->angular.x,(double)msg->angular.y,(double)msg->angular.z);

    boost::python::list a;
    a.append(msg->linear.x);
    a.append(msg->linear.y);
    a.append(msg->linear.z);

    boost::python::list b;
    b.append(msg->angular.x);
    b.append(msg->angular.y);
    b.append(msg->angular.z);

    //      boost::python::tuple b = boost::python::make_tuple(4, 5, 6);
    std::cout << "vars created\n" << std::flush;
    t_callback(a,b);
    //      t_callback(boost::python::make_tuple(msg->linear.x, msg->linear.y, msg->linear.z),
    //                 boost::python::make_tuple(msg->angular.x, msg->angular.y, msg->angular.z));

    std::cout << "after cb\n" << std::flush;
    //boost::python::PyGILState_Release(gstate);
    PyGILState_Release(gstate);
    std::cout << "after gil release\n" << std::flush;
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
  //  boost::python::Py_DECREF(t_callback);
}

void set_debug(int debug) // 0 -> no logs, not 0 -> logs
{
  dbg = debug;
}

void initialize(boost::python::object twist_callback) //give it a python function
{
  destruct = false;
  if (! PyEval_ThreadsInitialized()) {
    PyEval_InitThreads();
  }
  t_callback = twist_callback;
  int argc = 1; //fake an argc argv
  char *argv[] = { "woof", NULL};
  ros::init(argc, argv, "neato");
  //  ros::NodeHandle n;
  h = new ros::NodeHandle();
  init_laser_msg();
  init_odom_msg();
  init_transform();
  //  boost::python::Py_INCREF(twist_callback);
    std::cout << "Initialized";
  l_pub = h->advertise<sensor_msgs::LaserScan>("/base_scan", 10);  
  o_pub = h->advertise<nav_msgs::Odometry>("odom", 10);

  //subscribe to "cmd_vel", twist
  //ros::Subscriber sub = h->subscribe("/cmd_vel", 100, pass_t_callback);
  sub = h->subscribe("/cmd_vel", 100, pass_t_callback);
  spin_thread = boost::thread(ros_spin);  
}



void init_laser_msg()
{
  //do header
  laser_scan_msg.header.seq = 1; //TODO
  laser_scan_msg.header.frame_id = "scan_link"; //TODO
  laser_scan_msg.angle_min = 0.0f;
  laser_scan_msg.angle_max = 6.26f;
  laser_scan_msg.angle_increment = 0.017437326f;
  laser_scan_msg.range_min = 0.020f;
  laser_scan_msg.range_max = 5.0f;
}

void init_odom_msg()
{
  odom_msg.header.seq = 1; //TODO
  odom_msg.header.frame_id = "odom"; //TODO
  odom_msg.child_frame_id = "base_link"; //TODO
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation.x = 0;
  odom_msg.pose.pose.orientation.y = 0;
  odom_msg.twist.twist.linear.y = 0;
  odom_msg.twist.twist.linear.z = 0;
  odom_msg.twist.twist.angular.x = 0;
  odom_msg.twist.twist.angular.y = 0;
}

void init_transform()
{
  old_left = 0;
  old_right = 0;
  x = 0;
  y = 0;
  theta = 0;
  old_time = ros::Time::now();
}



BOOST_PYTHON_MODULE(neato_ros_cpp)
{
  using namespace boost::python;
  def("init", initialize);
  def("pub_laser", pub_laser);
  def("pub_transform", pub_transform);
  def("set_debug", set_debug);
  def("dispose", dispose);
}
