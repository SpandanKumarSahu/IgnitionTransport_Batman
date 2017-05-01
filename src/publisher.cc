#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

/// \brief Flag used to break the publisher loop and terminate the program.
static std::atomic<bool> g_terminatePub(false);

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that publishes messages
/// and exit the program smoothly.
void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM)
    g_terminatePub = true;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT,  signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Create a transport node and advertise a topic.
  ignition::transport::Node node;
  std::string topic = "/pose1";

  auto pub = node.Advertise<ignition::msgs::Pose>(topic);
  if (!pub)
  {
    std::cerr << "Error advertising topic [" << topic << "]" << std::endl;
    return -1;
  }

  // Prepare the message.
  ignition::msgs::Pose msg;
  ignition::msgs::Quaternion quat;
  ignition::msgs::Vector3d vec;
  double x = 0, y = 0, z = 0;
  vec.set_x(x);
  vec.set_y(y);
  vec.set_z(z);
  double xq = 0, yq = 0, zq = 0, wq = 0;
  quat.set_x(xq);
  quat.set_y(yq);
  quat.set_z(zq);
  quat.set_w(wq);
  msg.set_allocated_position(&vec);
  msg.set_allocated_orientation(&quat);

  // Publish messages at 1Hz.
  while (!g_terminatePub)
  {

	// vec.set_x(++x);
	// vec.set_y(y);
	// vec.set_z(z);
	// quat.set_x(xq);
	// quat.set_y(yq);
	// quat.set_z(zq);
	// quat.set_w(wq);
	// msg.set_allocated_position(&vec);
	// msg.set_allocated_orientation(&quat);
    if (!pub.Publish(msg))
      break;

    std::cout << "Publishing hello on topic [" << topic << "]" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}
