/*
  Listens to ROS on topic : "wheel_odometry"
  Publishes to ignition on : "/pose1"
  Author : Spandan Kumar Sahu
  Email : spandankumarsahu(at)gmail(dot)com
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

static ignition::msgs::Pose msg;
static ignition::msgs::Quaternion quat;
static ignition::msgs::Vector3d vec;
static std::atomic<bool> g_terminatePub(false);

void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM)
    g_terminatePub = true;
}

void chatterCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  vec.set_x(msg.position.x);
  vec.set_y(msg.position.y);
  vec.set_z(msg.position.z);
  quat.set_x(msg.orientation.x);
  quat.set_y(msg.orientation.y);
  quat.set_z(msg.orientation.z);
  quat.set_w(msg.orientation.w);
}

int main(int argc, char **argv)
{
  //ROS
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("wheel_odometry", 1000, chatterCallback);
  ros::spin();

  //IGNITION_TRANSPORT
  std::signal(SIGINT,  signal_handler);
  std::signal(SIGTERM, signal_handler);
  ignition::transport::Node node;
  std::string topic = "/pose1";

  auto pub = node.Advertise<ignition::msgs::Pose>(topic);
  if (!pub)
    {
      std::cerr << "Error advertising topic [" << topic << "]" << std::endl;
      return -1;
    }
  while (!g_terminatePub)
    {
      msg.set_allocated_position(&vec);
      msg.set_allocated_orientation(&quat);
      if (!pub.Publish(msg))
	break;
      std::cout << "Publishing hello on topic [" << topic << "]" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  return 0;
}


