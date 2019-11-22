#include "ros/ros.h"
#include "motor_control/coordinatePass.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coordinatePassClient");

  ros::NodeHandle node;
  ros::ServiceClient client = node.serviceClient<motor_control::coordinatePass>("coordinatePass");
  motor_control::coordinatePass srv;
  srv.request.request = true;
  client.call(srv);
  
  std::cout << srv.response.xCoordinate << std::endl;
  std::cout << srv.response.yCoordinate << std::endl;
  std::cout << srv.response.zCoordinate << std::endl;

  return 0;
}