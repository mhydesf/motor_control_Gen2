#include "ros/ros.h"
#include "motor_control/coordinatePass.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coordinatePassClient");
  if (argc != 3)
  {
    ROS_INFO("Usage: Receiving X Y Z coordinate ... ");
    return 1;
  }

  ros::NodeHandle node;
  ros::ServiceClient client = node.serviceClient<motor_control::coordinatePass>("coordinatePass");
  motor_control::coordinatePass msg;
  msg.request = true;
  if (client.call(msg))
  {
    ROS_INFO("%ld", (long int)msg.response.xCoordinate, (long int)msg.response.yCoordinate, (long int)msg.response.zCoordinate);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}