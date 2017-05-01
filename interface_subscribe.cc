/*
  Listens to Ignition on : "/pose"
  Publishes to ROS on : "pose_ignition"
  Author : Spandan Kumar Sahu
  Email : spandankumarsahu(at)gmail(dot)com
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

static ignition::msgs::Quaternion quat;
static ignition::msgs::Vector3d vec;
static geometry_msgs::Pose msg;

void cb(const ignition::msgs::Pose &_msg)
{
  vec = _msg.position();
  quat = _msg.orientation(); 
}

int main(int argc, char **argv)
{
  //IGNITION TRANSPORT
  ignition::transport::Node node;
  std::string topic = "/pose";

  if (!node.Subscribe(topic, cb))
  {
    std::cerr << "Error subscribing to topic [" << topic << "]" << std::endl;
    return -1;
  }

  //ROS
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("pose_ignition", 1000);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    msg.position.x = vec.x();
    msg.position.y = vec.y();
    msg.position.z = vec.z();
    msg.orientation.x = orientation.x();
    msg.orientation.y = orientation.y();
    msg.orientation.z = orientation.z();
    msg.orientation.w = orientation.w();

    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  ignition::transport::waitForShutdown();

  return 0;
}
