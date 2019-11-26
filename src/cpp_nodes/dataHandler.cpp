#include "ros/ros.h"
#include "motor_control/coordinatePass.h"
#include "dataHandler.h"

DataHandle::DataHandle(){

}

void DataHandle::requestCoordinate(){

  ros::NodeHandle node;
  ros::ServiceClient client = node.serviceClient<motor_control::coordinatePass>("coordinatePass");
  motor_control::coordinatePass srv;
  srv.request.request = true;
  client.call(srv);

  std::cout << srv.response.xCoordinate << std::endl;
  std::cout << srv.response.yCoordinate << std::endl;
  std::cout << srv.response.zCoordinate << std::endl;
}

DataHandle::~DataHandle(){
}

//
//  #           NOTE:             #
//  #   TESTING CLASS FUNCTION    #
//  # SCRIPT SHOULD NOT CALL MAIN #
//

int main(int argc, char **argv){
  
  ros::init(argc, argv, "coordinatePassClient");
  DataHandle client;

  client.requestCoordinate();

  return 0;
}